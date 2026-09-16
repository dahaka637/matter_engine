# Ideia do roadmap para animação do ragdoll ativo

Até o momento as nossas tentativas de fazer a animação totalmente por biomecânica, juntando equilibrio e movimentação falharam catastroficamente, e portanto, decidi por mudar a dinamica de como irei proceder essa etapa do desenvolvimento.

Fazer a movimentação totalmente por biomecânica sem nenhum auxilio externo ou preset é um problema atual até mesmo de grandes laboratórios de pesquisa e desenvolvimento em robótica avançada, não há como eu desenvolver isso sozinho de uma maneira rápida e "fácil". Desarte, vou me submeter aos métodos já consolidados na industria de games que utilizam-se de ragdoll ativo com animações procedurais 

### ETAPAS

O desenvolvimento desse novo sistema será divido em 4 etapas: 

- Consolidação do footwork: Fazer o footwork operar na movimentação de forma consolidada, e servir perfeitamente para mecânica de compensação de movimento para o caso de equilibrio;

- Equilibrio consolidado: Fazer o esqueleto conseguir utilizar-se perfeitamente do sistema de footowork para manter o equilibrio, bem como os movimentos de demais partes do corpo para compensação de impactos e movimentos (Nessa etapa, também estaremos implementando o sistema de auxilio de equilibrio externo, em que será aplicado uma força ou qualquer forma de ajuda principalmente no osso da bacia e um pouco nos ossos da coluna, para auxiliar o ragdoll ativo a manter o equilibrio de forma mais facilitada, sem que seja descartado o sisteam de equilibrio biomecânico, apens adaptado para funcionar em conjunto, dessa forma permitindo que as animações e movimentos ocorram de forma muito mais fluidas, agressvidas e organicas, sem se preucupar tanto em manter o equilibrio)

- Criação do laboratório de animações: um laboratório proprio de animações para testar viusalmente as animações que serão geradas no KIMODO;

- Implementação de animações geradas no ragdoll, com proceduralidade pelo footwork+sistmea de quilibrio dinamico.

``Nota: No nosso novo modelo de desenvolvimento, iremos utilizar animações já definidas, feitas no KIMODO e integrar essas animações ao nosso sisteam proprio, mesclando elas com o sistema de equilibrio e footwork``

## Etapa 1 - Rework do Footwork:

Nessa etapa, vamos refazer o footwork, agora, ele vai ser um footwork fisico, ou seja, vamos fazer apeans os ossos da cintrua para baixo, sendo eles os pés, ossos das pernas e bacia. Na bacia, vamos colocar uma "força mágica" que permite ela manter o equilibrio automáticamente.

Eu devo poder acessar o footwork através do menu de footwork que agora será totalmente fiscio, ou seja ele spawna o meio esqueleto, e eu devo conseguir me movimentar em todas as direçõse WASD (+direção do mouse) pelo footwork, em que ele agora realiza movimentos fisicos reais com os pés, através dos motores das juntas.
Eu também devo conseguir spawnar o *Meio esqueleto do footwork* através do menu de spawn de objetos. em que ele spawna o meio esqueleto já com o sistema de footkwrok ativado mas automatico, a ideia desse ai é eu conseguir manipular a bacia com a physcgun, e os pés devem predizer a direção e realziar a movimentação para a companhar a bacia automaticamente, como uma "tentativa de atualização de posição" mas de forma com que isso futuramente já sirva para auxilair no sistema de equilibrio futuramente (se o personagem é empurrado para a a direita, os pés já começam a preizer a direção do impulso e ir para direita para evitar do corpo cair)

Feito isso já teremos resolvido já metade do problema de equilibrio.

## Etapa 2 - Consolidação do sistema de equilibrio natural:

Após o footwork estar funcionando perfeitamente, poderemos então consolidar o sistema de equilibrio, com severos testes e calibrações, nessa etapa, o personagem 

Deve conseguir se manter perfeitamente equilibrado parado: sabendo rotacionar os ossos de seu corpo da cintura para cima para ficar reto E compensar impactos (Exemplo, está caindo para direita, tenta virar para a esquerda para compensar, mas obviamente de uma forma dinamica calculada) bem como saber usar o footwork para equilibrio (Exemplo: se está caindo para traś ele não deve ficar estatico, os pés devem tentar anteceder e caminhar rapidamente para trás para compensar a queda e tentar se manter em pé (apenas exemplo) ), e é claro, além disso tudo, também teremos o auxilio da "força mágica" na bacia, e UM POUCO na coluna, porém ela deve ter momentos de redução, por exemplo quando o personagem obviamente já caiu, pois se não ele ficaria com uma posição muito explicitamente artificial, parecendo estar levitando ou coisa assim, ou seja, a nossa "força mágica" terá momentos  em que estará mais fortes e outros em que estará muito fraca, para dar uma sensação de realismo, sem que ela seja visualmente perceptivel. Futuramente, após a conclusão da etapa 4, será implementaodo as animações de levantar (com sisteam de identificação de posiçõa base de caimento) para o personagem conseguir se levantar do chão após a queda, executando uma animação predefinida COM auxilio da "força mágica" na bacia e coluna para o colocar em pé mais fácilmente.

## Etapa 3 - Sistema de animação pre-definido:

Nessa etapa nós iremos realizar a ciração do laboratório de animações, um laboratório semelhante ao "visualizador de objetos" que permite mostrar isoladamente o RAGDOLL em um ambiente sem fisica e gravidade, e selecionar uma animação importada (Deverá ser desenvolvido também um sistema de importação das animaçõe). As animações serão realizadas no KIMODO (Sistema baseado em IA que me permite criar animações através de input textual, porém vou ter que estudar como ele exporta as animações), será gerado inicialmente as aninações de: Caminhada, trote (corrida leve), e corrida (corrida rápida, sprint), levantar-se do chão de costas, virar para o lado deitado, levantar-se do chão de frente.

### Etapa 4 - Implementação das animações e junção dos sistemas

Nessa etapa, final, nós iremos    fazer a junção de todos os sistemas que desenvolvemos nas etapas anteriores, seja eles: Sisteam de footwork, equilibrio e animações pré-definidas, tudo em um só, ajutadas é claro, e nesse teste o nosso ragdoll ativo deverá ser capaz de conseguir caminhar, correr e levantar-se de forma natural e eficiente, sem parecer robótico ou apresnetar comportamentos estranhos, ou que a "Força mágica" seja viusalmente perceptivel. Eu devo conseguir arrremessar objetos nele para desequilibra-lo e ele conseguir manter o equilibrio, bem como levanta-se quando cair.

### FINAL

Após tudo isso, então será trabalhado o sistema de manipulação de bola enquanto em movimento, em que proceduramnete o ragdoll ativo deverá conseguir manipular a bola de futebol com os pés, driblando e chutando-a, mas isso só começara a ser desenvolvido após sucesso absoluto nas estapas anteriores e termos um sistema de animação com ragdoll ativo bem consolidado e eficiente (Conseguir permitir mais de 50 ragdolls ao mesmo tempo executnado suas animações, equilibrio e tudo mais com performance boa)





# O que iremos fazer agora?

Bem, agora que você tem noção de qual caminho iremos percorer, claro que eu deixo CONTIGO a melhor definição dessa mecânicas, estaremos fando o Rework do Footwork, em que agora eu devo conseguir spawnar o meio esqueleto com footwork automatico, bem como agora ao entrar no footkwork eu passo a controlar ifsicament eo meio esqueleto. Só iremos as etapas posteriores após eu testar e provar manualmente essa parte.



### Nota:

Eu e o claude code tentamos em outra ocasião calibrar a movimentação bem como introduzir a ideia de força magica para auxiliar no equilibrio, mas foi CATASTRÓFICO e  a "força mágica" ficou abusrdamente ruim e bizzara, fazendo o eprsonagem ao cair ficar levitando com as pernas para o lado e etc.
