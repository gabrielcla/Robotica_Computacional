# Robotica_Computacional
Atividades para entrega da disciplina Robótica Computacional 2019.1
## Atividade 1
Entregável 1 - Visão Computacional
* Análise e equalização de histogramas dos tons da imagem.
* Separação de canais RGB da imagem com seus respectivos histogramas.
* Rearranjo dos pixels de uma imagem.
## Atividade 2
Entregável 2 - Visão Computacional
* Identificação de círculos com a cor pre selecionada de um frame.
* Cálculo da distância entre a folha com círculos e câmera do computador.
* Cálculo do ângulo da linha que conecta os dois círculos da folha com a horizontal.
* Desenho de centro e contorno sobre círculos de cor específicas (filtragem), usando transformada de Hough.
* Identificação de imagens/escritas específicas em um frame por meio da técnica SIFT.
## Atividade 3
* Reconhecimento de objetos com Deep Learning e OpenCV
* Rastreamento de objeto identificado em mais que 5 frames seguidos. ([vídeo](https://youtu.be/UlR5qervZN4))
* Comandos introtudórios para comandar um robô ROS Turtlebot.

## Projeto 1
* Código prinpcial com as funções e comandos para o robô: main.py <br />
* [Vídeo do funcionamento do robô](https://youtu.be/hs9sVoCzdCU) <br />

* O objeto a ser evitado pode ser escolhido via input limitado a uma lista pela rede neural utilizada (MobileNet Detection)
  * Contém um sistema de tracking para identificar o objeto escolhido somente após sua identificação em 5 frames seguidos.

* A cor a ser seguida pelo robô é azul, identificada pelo OpenCV.
  * Contém um controle proporcional para caso esteja se aproximando da cor. Assim, a medida que vai chegando mais perto da cor detectada pela câmera, vai diminuindo sua velocidade.

* O robô tem características de sobrevivência.
  * Um escaneamento infravermelho para evitar colisões com obstáculos próximos ao seu redor.
  * Bumpers para, caso haja colisão, se movimentar para desviar do obstáculo.
