<h1>
    <a href="docs/demo.gif"><img
        src="docs/demo.gif"
        alt="Demo GIF" width="300"></a>
</h1>

Controle preditivo de um ciclista em diferentes cenários de teste. Esse
repositório contém o código para simular e controlar um ciclista em um ambiente
3D. O controle é realizado através de um controlador preditivo baseado em
modelo (**MPC**) que utiliza um modelo dinâmico do ciclista para prever o
comportamento futuro e otimizar o controle.

Essa implementação foi feita para o meu trabalho de conclusão de curso (TCC)
como requisito para graduação em Ciência da Computação pela Universidade
Federal de Juiz de Fora.

---

### Conteúdo

1.  [Sobre o Projeto](#sobre-o-projeto)
2.  [Funcionalidades](#funcionalidades)
3.  [Requisitos](#requisitos)
4.  [Como Executar](#como-executar)
5.  [Estrutura do Projeto](#estrutura-do-projeto)
6.  [Resultados](#resultados)
7.  [Contribuição](#contribuição)
8.  [Licença](#licença)
9.  [Contato](#contato)

---

### Sobre o Projeto

O controle de um ciclista em um ambiente de simulação é um problema complexo,
que envolve a dinâmica não linear da bicicleta, a interação com o ambiente
(terreno, obstáculos ) e a necessidade de manter o equilíbrio e seguir
trajetórias desejadas.

Este projeto aborda esse desafio utilizando **Controle Preditivo Baseado em
Modelo (MPC)**. O MPC permite:

* **Antecipar o comportamento futuro:** Usando um modelo dinâmico da bicicleta
e do ciclista, o controlador prevê como o sistema se comportará nos próximos
instantes.
* **Otimizar o controle:** Com base nas previsões, o MPC calcula as ações de
controle ótimas (como torque aplicado aos pedais e ângulo de direção) para
atingir os objetivos (manter o equilíbrio, seguir uma rota, evitar obstáculos),
respeitando as restrições do sistema (limites de torque, ângulo de
esterçamento).
* **Adaptabilidade:** O MPC pode lidar com mudanças nas condições e
perturbações do ambiente.

O motor de física [MuJoCo](https://github.com/google-deepmind/mujoco) foi
utilizado para simular a dinâmica do ciclista e da bicicleta. O MuJoCo é um
motor de física projetado para simulações de robótica e oferece um bom
equilíbrio em performance e precisão. Também foi utilizado o [MuJoCo
MPC](https://github.com/google-deepmind/mujoco_mpc), que disponilizada um
framework para trabalhar com MPC e a MuJoCo.

---

### Funcionalidades

* Modelo dinâmico detalhado da bicicleta e ciclista.
* Implementação de Controlador Preditivo Baseado em Modelo (MPC).
* Simulação 3D do ciclista em diferentes terrenos e condições.
* Capacidade de definir trajetórias de referência.
* Visualização de dados da simulação (posição, velocidade, ângulos, ações de controle).
* Cenários de teste configuráveis.

---

### Requisitos

Para executar o projeto, é necessário compilar o MuJoCo MPC, disponível em
[https://github.com/google-deepmind/mujoco_mpc](https://github.com/google-deepmind/mujoco_mpc).
As instruções detalhadas de como compilar estão disponíveis lá. Só foi testada
a versão 0.1 do MuJoCo MPC.

Esse projeto só foi testado no Linux, mas deve funcionar no Windows e MacOS. É
recomendável uma máquina com pelo menos 6 núcleos e tecnologia Hyper-Threading
ou equivalente, dada a natureza computacionalmente intensiva do MPC.

Após configurar o MuJoCo MPC, é necessário incluir os arquivos desse projeto, e
adicionar a tarefa.

### Licença

Este projeto é licenciado sob a licença MIT. Veja o arquivo [LICENSE](LICENSE)
para mais detalhes.

### Contato

Vítor Mouro - [vitor.a.mouro@gmail.com](mailto:vitor.a.mouro@gmail.com)
