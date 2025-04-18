#include "mjpc/tasks/bicycle/bicycle.h"

#include <cmath>
#include <string>
#include <format>
#include <fstream>

#include <mujoco/mujoco.h>
#include <absl/flags/flag.h>

#include "absl/flags/declare.h"
#include "mjpc/task.h"
#include "mjpc/utilities.h"
#include "mjpc/simulate.h"
#include "GLFW/glfw3.h"
#include "path.h"

ABSL_DECLARE_FLAG(std::string, output_file);

extern std::unique_ptr<mujoco::Simulate> sim;

namespace mjpc
{
    std::string Bicycle::XmlPath() const
    {
        const std::string path = "bicycle/experiments/task.xml";
        return GetModelPath(path);
    }

    void Bicycle::printInfo() {
        std::cout << std::endl;
        for (int i = 0; i < parameters.size(); i++)
            std::cout << std::format("Parameter {}: {}\n", i, parameters[i]);
        for (int i = 0; i < weight.size(); i++)
            if (weight[i] != 0)
                std::cout << std::format("Weight: {}: {}\n", i, weight[i]);
        std::cout << std::endl;
    }

    std::string Bicycle::Name() const { return "Bicycle"; }

    Bicycle::Bicycle() : residual_(this) {
        path_ = new Path(50);

        // Load points from csv (9 doubles per line)
        std::string path_csv = "mjpc/tasks/bicycle/experiments/path.csv";
        int res = path_->loadFromFile(path_csv);
        if (res != 0)
            mju_error("Failed to load path from %s", path_csv.c_str());

        metrics = new Metrics(path_->getCurve().size() / 3);

        std::string outfile = absl::GetFlag(FLAGS_output_file);
        out.open(outfile, std::ios::binary);
        if (!out.good())
            mju_error("Failed to open output file %s", outfile.c_str());
    }

    Bicycle::~Bicycle()
    {
        delete path_;
        delete metrics;
        out.close();
    }

    int GetVelocityGoal(mjtNum *vel, mjtNum *head)
    {
        int count;
        const float *axes = glfwGetJoystickAxes(GLFW_JOYSTICK_1, &count);
        if (count < 5)
            return 0;

        float heading = -axes[0] * M_PI;
;
        if (head != nullptr)
            *head = heading;
        float max_speed = 5;
        float speed = (axes[4] + 1) / 2 * max_speed;

        if (vel != nullptr)
        {
            vel[0] = speed * cos(heading);
            vel[1] = speed * sin(heading);
            vel[2] = 0;
        }
        return 1;
    }

    void ActionResidual(const mjModel *model, const mjData *data, double *residual, int *counter)
    {
        int humanoid_controls = 21;
        mjtNum *start = data->ctrl + model->nu - humanoid_controls;
        mju_copy(&residual[*counter], start, humanoid_controls);
        *counter += humanoid_controls;
    }

    void PoseResidual(const mjModel *model, const mjData *data, double *residual, int *counter)
    {
        // Arms are at last 6 positions of qpos
        mjtNum arms[6] = {0.477525, -0.31974, -0.750274, 0.477525, -0.31974, -0.750274};
        mjtNum arms_error[6];
        mju_sub(arms_error, data->qpos + model->nq - 6, arms, 6);
        mju_copy(&residual[*counter], arms_error, 6);
        *counter += 6;

        // Abdomen is at last model-nq - 21 to model-nq - 18
        mjtNum abdomen[3] = {0.0, -0.26, 0.0};
        mjtNum abdomen_error[3];
        mju_sub(abdomen_error, data->qpos + model->nq - 21, abdomen, 3);
        mju_copy(&residual[*counter], abdomen_error, 3);
        *counter += 3;
    }

    void VelocityResidual(const mjModel *model, const mjData *data, double *residual, int *counter,
        const std::vector<double> &parameters_)
    {
        double speed_goal = parameters_[0];
        double heading_goal = -parameters_[1]; // In radians [-pi, pi]

        double target_velocity[3] = {speed_goal * cos(heading_goal),
                                     speed_goal * sin(heading_goal), 0};

        GetVelocityGoal(target_velocity, nullptr);

        double *currect_velocity = SensorByName(model, data, "frame_subtreelinvel");
        double velocity_error[3];
        mju_sub3(velocity_error, target_velocity, currect_velocity);
        double velocity_error_norm = mju_norm3(velocity_error);
        residual[(*counter)++] = velocity_error_norm;
    }

    void BalanceResidual(const mjModel *model, const mjData *data, double *residual, int *counter)
    {
        mjtNum *up_axis = SensorByName(model, data, "bicycle_yaxis");
        residual[(*counter)++] = up_axis[2] - 1.0;
    }

    void PositionResidual(const mjModel *model, const mjData *data, double *residual, int *counter)
    {
        mjtNum *goal_pos = SensorByName(model, data, "goal_pos");
        mjtNum *bicycle_pos = SensorByName(model, data, "bicycle_pos");
        mjtNum goal_displacement[3];
        mju_sub3(goal_displacement, goal_pos, bicycle_pos);
        mjtNum goal_distance = mju_norm3(goal_displacement);
        residual[(*counter)++] = goal_distance;
    }

    void GoalResidual(const mjModel *model, const mjData *data, double *residual, int *counter,
        const std::vector<double> &parameters_)
    {
        // The bicycle should reach the goal position at a certain speed and heading
        mjtNum *goal_pos = SensorByName(model, data, "goal_pos");
        mjtNum *bicycle_pos = SensorByName(model, data, "bicycle_pos");

        mjtNum goal_displacement[3];
        mju_sub3(goal_displacement, goal_pos, bicycle_pos);
        goal_displacement[2] = 0; // Ignore the z-axis
        mjtNum goal_distance = mju_norm3(goal_displacement);
        residual[(*counter)++] = goal_distance;

        mjtNum goal_speed = parameters_[0];
        mjtNum *goal_xaxis = SensorByName(model, data, "goal_zaxis");
        mjtNum goal_velocity[3];
        mju_scl3(goal_velocity, goal_xaxis, goal_speed);
        mjtNum *bicycle_velocity = SensorByName(model, data, "frame_subtreelinvel");
        mjtNum velocity_error[3];
        mju_sub3(velocity_error, goal_velocity, bicycle_velocity);
        residual[(*counter)++] = mju_norm3(velocity_error);
    }

    int getClosestPoint(const mjModel *model, const mjData *data, const Path *path, const int current_point_i)
    {

        std::vector<double> curve = path->getCurve();

        // Closest point on curve
        // mjtNum *bicycle_pos = SensorByName(model, data, "bicycle_pos");
        mjtNum *bicycle_pos = SensorByName(model, data, "track_pos");
        int closest_point_i = current_point_i;
        double closest_point[3];
        closest_point[0] = curve[current_point_i * 3];
        closest_point[1] = curve[current_point_i * 3 + 1];
        closest_point[2] = curve[current_point_i * 3 + 2];
        for (int i = closest_point_i + 1; i < curve.size() / 3; i++)
        {
            double current_point[3] = {curve[i * 3], curve[i * 3 + 1], curve[i * 3 + 2]};
            mjtNum dist = mju_dist3(bicycle_pos, current_point);
            mjtNum cur_distance = mju_dist3(bicycle_pos, closest_point);
            if (cur_distance >= dist)
            {
                closest_point[0] = curve[i * 3];
                closest_point[1] = curve[i * 3 + 1];
                closest_point[2] = curve[i * 3 + 2];
                closest_point_i = i;
            } else {
                break;
            }
        }

        return closest_point_i;
    }

    void PathResidual(const mjModel *model, const mjData *data, double *residual, int *counter,
        const std::vector<double> &parameters_, const Path *path, int current_point_i)
    {
        std::vector<double> curve = path->getCurve();

        // Closest point on curve
        // mjtNum *bicycle_pos = SensorByName(model, data, "bicycle_pos");
        mjtNum *bicycle_pos = SensorByName(model, data, "track_pos");
        int closest_point_i = getClosestPoint(model, data, path, current_point_i);
        double closest_point[3] = {curve[closest_point_i * 3], curve[closest_point_i * 3 + 1], curve[closest_point_i * 3 + 2]};

        mjtNum dist = mju_dist3(bicycle_pos, closest_point);
        residual[(*counter)++] = dist;

        // Velocity target on the point
        mjtNum target_speed = parameters_[0];
        double p0[3], p1[3];
        double t = (double)closest_point_i / (curve.size() / 3) * (path->getNumAnchors() - 1);
        double k = 0.01;
        if (closest_point_i == 0)
        {
            p0[0] = closest_point[0];
            p0[1] = closest_point[1];
            p0[2] = closest_point[2];
        }
        else
        {
            path->getPoint(p0, t - k);
        }
        if (closest_point_i == curve.size() / 3 - 1)
        {
            p1[0] = closest_point[0];
            p1[1] = closest_point[1];
            p1[2] = closest_point[2];
        }
        else
        {
            path->getPoint(p1, t + k);
        }
        // Unit vector between points
        double vel[3];
        mju_sub3(vel, p1, p0);
        mju_normalize3(vel);
        mju_scl3(vel, vel, target_speed);

        // Calculate velocity residual
        mjtNum *current_vel = SensorByName(model, data, "frame_subtreelinvel");
        mjtNum velocity_error[3];
        mju_sub3(velocity_error, current_vel, vel);
        residual[(*counter)++] = mju_norm3(velocity_error);
    }

    void Bicycle::ResidualFn::Residual(const mjModel *model, const mjData *data,
                                       double *residual) const
    {
        int counter = 0;

        // PositionResidual(model, data, residual, &counter);
        // VelocityResidual(model, data, residual, &counter, parameters_);
        // BalanceResidual(model, data, residual, &counter);
        ActionResidual(model, data, residual, &counter);
        // PoseResidual(model, data, residual, &counter);
        // GoalResidual(model, data, residual, &counter, parameters_);
        const Bicycle *task = dynamic_cast<const Bicycle *>(task_);
        PathResidual(model, data, residual, &counter, parameters_, task->getPath(), task->current_point_i);

        int user_sensor_dim = 0;
        for (int i = 0; i < model->nsensor; i++)
            if (model->sensor_type[i] == mjSENS_USER)
                user_sensor_dim += model->sensor_dim[i];
        if (user_sensor_dim != counter)
            mju_error_i(
                "mismatch between total user-sensor dimension "
                "and actual length of residual %d",
                counter);
    }

    void Bicycle::ModifyScene(const mjModel *model, const mjData *data, mjvScene *scene) const
    {
        // Draw path segments ------------------------------------------------------------------------------------------
        float segment_color[4] = {1.0, 0.0, 1.0, 1.0};
        double zero3[3] = {0};
        double zero9[9] = {0};
        float width = 0.01;
        std::vector<double> curve = path_->getCurve();
        int n_points = curve.size() / 3;

        for (size_t i = current_point_i; i < n_points - 1; i++)
        {
            // check max geoms
            if (scene->ngeom >= scene->maxgeom)
            {
                printf("max geom!!!\n");
                continue;
            }

            // initialize geometry
            mjv_initGeom(&scene->geoms[scene->ngeom], mjGEOM_CAPSULE, zero3, zero3, zero9,
                         segment_color);

            // make geometry
            double a[3], b[3];
            a[0] = curve[i * 3 + 0];
            a[1] = curve[i * 3 + 1];
            a[2] = curve[i * 3 + 2];
            b[0] = curve[(i + 1) * 3 + 0];
            b[1] = curve[(i + 1) * 3 + 1];
            b[2] = curve[(i + 1) * 3 + 2];

            mjv_makeConnector(
                &scene->geoms[scene->ngeom], mjGEOM_CAPSULE, width,
                a[0], a[1], a[2], b[0], b[1], b[2]);
            // increment number of geometries
            scene->ngeom += 1;
        }

        // Draw the anchors --------------------------------------------------------------------------------------------
        float anchor_color[4] = {0.0, 1.0, 0.0, 1.0};
        int num_anchors = path_->getNumAnchors();
        for (int i = 0; i < num_anchors; i++)
        {
            double pos[3];
            path_->getAnchor(pos, i);
            double size[3] = {width*2, width*2, width*2};
            AddGeom(scene, mjGEOM_SPHERE, size, pos, nullptr, anchor_color);

            double ctl_left[3], ctl_right[3];
            double ctl_size[3] = {width, width, width};
            float ctl_color[4] = {0.0, 1.0, 1.0, 1.0};
            path_->getLeftControl(ctl_left, i);
            path_->getRightControl(ctl_right, i);
            AddGeom(scene, mjGEOM_SPHERE, ctl_size, ctl_left, nullptr, ctl_color);
            AddGeom(scene, mjGEOM_SPHERE, ctl_size, ctl_right, nullptr, ctl_color);
            mjv_initGeom(&scene->geoms[scene->ngeom], mjGEOM_CAPSULE, zero3, zero3, zero9, ctl_color);
            mjv_makeConnector(
                &scene->geoms[scene->ngeom], mjGEOM_CAPSULE, width/2,
                ctl_left[0], ctl_left[1], ctl_left[2], ctl_right[0], ctl_right[1], ctl_right[2]);
            scene->ngeom += 1;
        }

        // Draw closest_point ------------------------------------------------------------------------------------------
        int closest_point_i = getClosestPoint(model, data, path_, current_point_i);
        double closest_point[3] = {curve[closest_point_i * 3], curve[closest_point_i * 3 + 1], curve[closest_point_i * 3 + 2]};
        const float c_color[4] = {0.0, 1.0, 0.0, 0.3};
        const double c_size[3] = {0.1, 0.1, 0.1};
        AddGeom(scene, mjGEOM_SPHERE, c_size, closest_point, nullptr, c_color);

        // Draw current and target velocity ----------------------------------------------------------------------------
        mjtNum *currentVel = SensorByName(model, data, "frame_subtreelinvel");
        mjtNum *bicycle_pos = SensorByName(model, data, "track_pos");
        mjv_initGeom(&scene->geoms[scene->ngeom], mjGEOM_ARROW, zero3, zero3, zero9, c_color);
        mjv_makeConnector(&scene->geoms[scene->ngeom], mjGEOM_ARROW, 0.05,
                          bicycle_pos[0], bicycle_pos[1], bicycle_pos[2],
                          bicycle_pos[0] + currentVel[0], bicycle_pos[1] + currentVel[1], bicycle_pos[2] + currentVel[2]);
        scene->ngeom += 1;

        // Velocity target on the point --------------------------------------------------------------------------------
        mjtNum target_speed = residual_.parameters_[0];
        double p0[3], p1[3];
        double t = (double)closest_point_i / (curve.size() / 3) * (path_->getNumAnchors() - 1);
        double k = 0.01;
        if (closest_point_i == 0)
        {
            p0[0] = closest_point[0];
            p0[1] = closest_point[1];
            p0[2] = closest_point[2];
        }
        else
        {
            path_->getPoint(p0, t - k);
        }
        if (closest_point_i == curve.size() / 3 - 1)
        {
            p1[0] = closest_point[0];
            p1[1] = closest_point[1];
            p1[2] = closest_point[2];
        }
        else
        {
            path_->getPoint(p1, t + k);
        }

        // Unit vector between points ----------------------------------------------------------------------------------
        double vel[3];
        mju_sub3(vel, p1, p0);
        mju_normalize3(vel);
        mju_scl3(vel, vel, target_speed);

        mjv_initGeom(&scene->geoms[scene->ngeom], mjGEOM_ARROW, zero3, zero3, zero9, c_color);
        mjv_makeConnector(&scene->geoms[scene->ngeom], mjGEOM_ARROW, 0.05,
                          closest_point[0], closest_point[1], closest_point[2],
                          closest_point[0] + vel[0], closest_point[1] + vel[1], closest_point[2] + vel[2]);
        scene->ngeom += 1;
    }

    void Bicycle::TransitionLocked(mjModel *model, mjData *data)
    {
        // Transmission ------------------------------------------------------------------------------------------------
        mjtNum *current_pos = SensorByName(model, data, "bicycle_pos");
        double tolerance = 0.5;
        mjtNum current_goal_pos[3];
        mju_copy3(current_goal_pos, data->mocap_pos);

        mjtNum goal_displacement[3];
        mju_sub3(goal_displacement, current_goal_pos, current_pos);
        mjtNum goal_distance = mju_norm3(goal_displacement);
        if (goal_distance < tolerance)
        {
            current_goal_pos[0] += 3;
            mju_copy3(data->mocap_pos, current_goal_pos);
        }

        // Update path -------------------------------------------------------------------------------------------------
        const auto now = steady_clock::now();
        const std::vector<double> curve = path_->getCurve();
        const int closest_point_i = getClosestPoint(model, data, path_, current_point_i);
        if (closest_point_i != current_point_i)
        {
            last_advance = now;
            if (start_time == time_point<steady_clock>::min())
                start_time = now;
        }
        current_point_i = closest_point_i;

        // Metrics -----------------------------------------------------------------------------------------------------
        double cur_pos[3], target_pos[3];
        memcpy(&cur_pos, SensorByName(model, data, "track_pos"), 3 * sizeof(double));
        memcpy(&target_pos, &curve[current_point_i * 3], 3 * sizeof(double));

        double current_distance = sqrt(pow(target_pos[0] - cur_pos[0], 2) + pow(target_pos[1] - cur_pos[1], 2));
        bool trajectoryUpdated = metrics->updateTrajectoryError(curve, current_point_i, current_distance);

        // SensorData --------------------------------------------------------------------------------------------------
        if (trajectoryUpdated) {
            double *site_sensor = SensorByName(model, data, "track_pos");
            if (!site_sensor)
                mju_error("track_pos: sensor not found");
            Point site = {site_sensor[0], site_sensor[1], site_sensor[2]};

            double *com_sensor = SensorByName(model, data, "frame_subtreecom");
            if (!com_sensor)
                mju_error("frame_subtreecom: sensor not found");
            Point com = {com_sensor[0], com_sensor[1], com_sensor[2]};

            double *xaxis_sensor = SensorByName(model, data, "bicycle_xaxis");
            double *yaxis_sensor = SensorByName(model, data, "bicycle_yaxis");
            double *zaxis_sensor = SensorByName(model, data, "bicycle_zaxis");
            if (!xaxis_sensor || !yaxis_sensor || !zaxis_sensor)
                mju_error("bicycle_axis: sensor not found");
            Point euler = {*xaxis_sensor, *yaxis_sensor, *zaxis_sensor};

            double *linvel_sensor = SensorByName(model, data, "frame_subtreelinvel");
            if (!linvel_sensor)
                mju_error("frame_subtreeinvel: sensor not found");
            Point linear = {linvel_sensor[0], linvel_sensor[1], linvel_sensor[2]};

            double *angvel_sensor = SensorByName(model, data, "frame_frameangvel");
            if (!angvel_sensor)
                mju_error("frame_frameangvel: sensor not found");
            Point angular = {angvel_sensor[0], angvel_sensor[1], angvel_sensor[2]};

            double control_efford = 0;
            for (int i = 0; i < model->nu; i++) {
                control_efford += abs(data->ctrl[i]);
            }
            control_efford /= model->nu;

            Point target_point = {target_pos[0], target_pos[1], target_pos[2]};

            auto duration = steady_clock::now() - start_time;
            double time = duration.count();

            metrics->updateTimeSeriesData(site, com, euler, linear, angular, target_point, control_efford, time);
        }


        // Task End Condition ------------------------------------------------------------------------------------------
        return;
        duration<double> time_since_advance = now - last_advance;
        bool timeout = time_since_advance > advance_timeout && last_advance > time_point<steady_clock>::min();
        bool goal_reached = current_point_i >= curve.size()/3 - 1;

        // Task if roll angle too big
        mjtNum *up_axis = SensorByName(model, data, "bicycle_yaxis");
        bool fail = up_axis[2] != 0 && up_axis[2] < 0.4;

        if ((timeout || goal_reached || fail) && sim->run) {
            metrics->updateTrajectoryTime(start_time, now);
            metrics->updateSuccessRate(current_point_i, curve.size()/3-1);
            // printInfo();
            metrics->print();
            metrics->writeSensorData(out);
            sim->exitrequest.store(true);
        }
    }

    void Bicycle::ResetLocked(const mjModel *model)
    {
        current_point_i = 0;
        metrics->reset();
        start_time = time_point<steady_clock>::min();
        last_advance = time_point<steady_clock>::min();
    }
} // namespace mjpc
