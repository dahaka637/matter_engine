# Animação alvo e assistência corporal — contrato de controle

Data: 15/09/2026. Substitui as tentativas de ancoragem mundial e de proibir Z.
Estado: contrato implementado; locomoção física completa ainda não aprovada.

## Requisito do usuário

A assistência pode agir em X, Y e Z. Ajuda o corpo a executar um movimento;
não o obriga a alcançar a posição que ele teria numa animação reproduzida no
mundo. Após arrastar/deslocar o corpo, não existe dívida de deslocamento a
compensar. Tampouco deve se tornar a sustentação principal ou impedir quedas.

Converter um erro mundial para coordenadas locais **não** resolve ancoragem.
`R^-1 * (p_mundial_salvo - p_atual)` ainda contém o ponto proibido. O erro
posicional mundial precisa desaparecer do objetivo, não apenas mudar de base.

## Separação adotada

1. **Motores:** clipe canônico → coordenadas articulares limitadas → drives.
   A simulação, os contatos e os limites continuam autoritativos a 120 Hz.
2. **Auxílio de pose:** erros dos membros relativos à pelve física atual.
   Não contém velocidade de translação/rotação rígida do corpo inteiro.
3. **Auxílio de movimento:** intenção de velocidade no referencial da frente
   corporal atual. Admite as três componentes; não integra velocidade para
   criar uma trajetória mundial obrigatória.
4. **Auxílio de equilíbrio:** pequeno torque de postura, usando a gravidade,
   orientação atual e inclinação da referência. Não segura um yaw mundial
   antigo nem uma posição anterior. Apoios atuais são contexto físico.

Itens 2–4 compartilham um orçamento de forças/torques, somado sobre todos os
links. Não se concede o orçamento completo a cada osso. Os valores iniciais
(12% do peso para soma das forças, 3,5% de peso × altura para soma dos torques)
são parâmetros provisórios, não leis biomecânicas nem garantias de qualidade.

## Referencial e derivadas

Sejam `p0`, `v0`, `R0`, `omega0` o centro de massa, velocidade, orientação e
velocidade angular da pelve física atual. Para o COM de cada link:

```text
r_i = R0^-1 * (p_i - p0)
u_i = R0^-1 * (v_i - v0) - (R0^-1 * omega0) × r_i
f_i = m_i * [kp * (r_alvo_i - r_i) + kd * (u_alvo_i - u_i)]
```

O FK da referência é construído diretamente em espaço corporal, com raiz
identidade. A derivada do alvo usa somente poses nesse mesmo espaço. Não
subtrair posições mundiais de ticks diferentes: isso reintroduz forças de
arraste quando o personagem é carregado ou deslocado. Os snapshots físicos
fornecem velocidades lineares dos COMs; considerar os offsets dos COMs nas
posições e nos braços de alavanca.

O auxílio de pose retira sua resultante de força por distribuição proporcional
à massa e cancela também o momento resultante, incluindo braços de alavanca.
Logo ele pode ajudar um pé a subir e outro segmento a reagir, mas não levantar
secretamente o corpo inteiro. Isso **não proíbe força vertical**: o termo de
movimento pode produzir resultante vertical explícita, dentro do orçamento.

O orçamento aplica um fator comum a forças e torques, preservando essa
separação. A rotina `computeBodyRelativeRagdollAssist3D` não guarda estado e
não recebe um destino posicional, origem de percurso ou altura mundial alvo.

## Aplicação no controlador atual

- Raiz mundial da pose de inspeção = raiz física **deste tick**.
- `m_commandOrigin` serve exclusivamente para medir distância na interface.
- Root motion fornece velocidade/cadência; não alimenta uma mola posicional.
- A freada considera a velocidade física de entrada, não atraso contra trilho.
- Neste primeiro ensaio, apoio carregado libera o auxílio líquido de
  locomoção. Isso é uma escolha para este teste, não uma proibição geral de
  auxílio em voo. O auxílio de pose continua tridimensional e sem resultante.
- Queda reduz a autoridade e depois suspende assistência. A Physgun suspende
  também motores e feedforward interno; a soltura reinicia na pose/local atuais.
- A compensação gravitacional do backend é **articular**: seus seis termos de
  raiz são descartados. Não equivale à antiga sustentação externa de 90% do peso.
- Ajustes ad hoc de tronco/tornozelo da tentativa anterior foram retirados.
  Não restaurar o footwork/WBC arquivado para esconder falhas desta etapa.

## Validação em camadas

`MatterEngine.BodyRelativeAssistance` cobre o contrato; não prova locomoção:

- transladar corpo e contexto em XYZ preserva as forças correspondentes;
- rotacionar em yaw rotaciona forças, sem recuperar uma direção mundial antiga;
- carregar o corpo com movimento rígido não vira erro de velocidade de pose;
- auxiliar a pose tem resultantes nulas de força e momento;
- comandos verticais positivos e negativos geram auxílio vertical;
- limites são globais e contam as reações, não apenas forças convenientes;
- mudar o histórico de posição durante corrida/freada não muda os comandos;
- soltar após arraste equivale a começar na pose atual, sem impulso de retorno;
- uma caixa física contra a canela desloca o corpo em relação ao controle.

`MatterEngine.AnimatedRagdoll` continua cobrando a corrida física completa.
Não retirar ou afrouxar esse teste para apresentar contratos algébricos como
sucesso de locomoção. Aprovação ainda exige Idle estável, passada compatível
com velocidade/apoiamento reais, freada, reação a impactos, limites e inspeção
das solas. O pé torto ainda precisa de diagnóstico físico/visual próprio.

Resultados em 15/09: contratos aprovados em Debug/RelWithDebInfo; caixa de
25 kg a 8 m/s contra a canela deslocou COM 0,368 m frente ao controle e provocou
queda. Sem chão, queda do COM de 1,238 m em 0,5 s. Suíte Debug 4/5: locomoção
completa reprovada por queda, sem afrouxar seu teste. O smoke do laboratório
confirmou essa limitação. Builds e fronteiras arquiteturais aprovados.

## Estudo utilizado e limites das referências

[SIMBICON (Yin, Loken e van de Panne, 2007)](https://www.cs.ubc.ca/~van/papers/2007-siggraph-simbicon.pdf)
separa acompanhamento de pose e feedback de equilíbrio, incluindo relação
COM–apoio e velocidade. Também discute diferenças entre movimento capturado e
dinâmica do personagem: retarget angular aprovado não garante marcha estável.

[DeepMimic (Peng et al., 2018)](https://arxiv.org/html/1804.02717)
usa características no referencial corporal e separa objetivos de imitação e
tarefa. Isso não significa que todos os seus objetivos sejam livres de alvos
mundiais, nem que adotar suas coordenadas reproduza um controlador treinado.

A assistência acima é uma construção explícita para o requisito deste jogo,
não uma implementação de SIMBICON/DeepMimic e não uma promessa de corrida
perfeita. Não há treinamento de política ou instalação dessas bibliotecas.
