# Estudo para footwork, equilíbrio e recuperação do ragdoll ativo

Data: 16/09/2026. Este documento distingue evidência, proposta e critérios de
aceitação. Estudar uma técnica ou passar testes de matemática não demonstra
que o dummy já consegue caminhar ou levantar fisicamente.

## Evidência utilizada

[SIMBICON, Yin, Loken e van de Panne, 2007](https://www.cs.sfu.ca/~kkyin/papers/Yin_SIG07.pdf)
combina poses articulares com uma máquina de estados e realimentação de
equilíbrio. Algumas transições dependem do contato real do pé. A posição e a
velocidade do COM relativamente ao apoio alteram a colocação da perna livre.
O artigo também distribui controle do tronco e quadris através de torques
internos. Isso fundamenta separar reprodução da animação e decisão de apoio;
não basta aumentar ganhos para converter mocap em marcha estável.

[DCM, Englsberger, Ott e Albu-Schäffer, 2014](https://elib.dlr.de/90244/1/JEnglsberger_DW14Abstract_Finalization_V2.pdf)
expõe a componente divergente como posição do COM acrescida de uma parcela
da velocidade. Ela permite raciocinar sobre a evolução do equilíbrio, além
da projeção estática do COM. A formulação inclui forças externas realizáveis
e transições de apoio. Adotar somente seu indicador de risco não implementa
o controlador completo nem prova estabilidade do nosso corpo articulado.

[Dafarra, Romano e Nori, 2017](https://arxiv.org/pdf/1705.10579)
usa capture point para escolher passos e gerar referências para um
controlador de momento com torques. O modelo simplificado pressupõe altura
constante do COM e contato de apoio; a formulação completa considera atrito,
centro de pressão e mudanças das restrições de contato. O paper dá suporte a
uma arquitetura em camadas, mas seus ganhos e resultados não são diretamente
transferíveis ao Crash Dummy.

[FRASA, Gaspard et al., 2024](https://arxiv.org/html/2410.08655v1)
trata recuperação no chão como um problema próprio, com novos contatos e
adaptação ao estado real. Compara sua política treinada a sequências de
keyframes; examina variação dos comandos, colisões e robustez. Não estamos
implementando nem treinando FRASA. Sua relevância aqui é mostrar por que um
clipe visual de levantar, sozinho, não garante recuperação física.

[Documentação PhysX de articulações](https://nvidia-omniverse.github.io/PhysX/physx/5.6.1/docs/Articulations.html)
define unidades diferentes para drives de força e de aceleração e explicita
o orçamento conjunto de esforço do motor via performance envelope. O SDK
local usado pelo projeto é 5.9.0; as definições foram confirmadas no checkout
em `build-linux/_deps/physx-src/physx/include/solver/PxSolverDefs.h` e
`PxArticulationReducedCoordinate.h`. A compensação gravitacional de uma base
flutuante contém seis termos de raiz e os termos articulares. Descartar os
seis primeiros evita uma sustentação externa implícita, mas os restantes
não resolvem, por si, a dinâmica de contato.

## Auditoria da base no início desta etapa

Estes pontos descrevem o código encontrado antes da reconstrução desta
etapa. Devem ser consultados como diagnóstico histórico, não como lista
permanente de defeitos ainda presentes.

- `AnimatedRagdollController3D::update` desligava todos os motores quando a
  Physgun agarrava o corpo. A flacidez era uma escolha explícita de controle,
  além de qualquer problema de força muscular.
- `Falling` era terminal: não havia classificação frente/costas nem rotina
  de recuperação. A força articular caía a 12% e o alvo anterior congelava.
- A fase da corrida vinha do relógio. A carga de contato só autorizava
  propulsão auxiliar; não corrigia o instante da troca de pés, posição da
  próxima passada ou transferência de peso.
- O perfil marcava somente os pés como sensores. Recuperação precisa também
  reconhecer contatos externos de mãos, joelhos, tronco e pelve.
- O callback de sensores aceitava contatos de um ragdoll consigo mesmo. Sem
  filtrá-los, um pé sobre a própria perna pode parecer um apoio no ambiente.
- `applyActiveRagdollFeedforward` somava gravidade e feedforward customizado
  sem limitar a soma com o drive. Limitar cada parcela separadamente não
  limita o músculo total. O envelope de esforço disponível no SDK oferece
  um caminho para tornar esse contrato verificável.
- Os multiplicadores globais de stiffness/damping do backend eram uma
  calibração provisória. Ausência de força, saturação e equilíbrio ruim são
  diagnósticos distintos; não se deve tratar todos elevando stiffness.

## Contrato proposto

O corpo físico atual é a referência de posição. Gravidade e contatos reais
com o ambiente são informações necessárias e legítimas. O que continua
proibido é uma mola que leva a raiz a uma posição mundial salva ou à posição
onde estaria caso tivesse seguido a animação sem perturbação. Um passo pode
usar a superfície de apoio atual; quando esse apoio deixa de existir, não
fica uma dívida posicional que puxe o personagem de volta.

O fluxo fixo continua: observar o resultado do passo anterior, decidir
apoios/fase, produzir alvos limitados e torques, simular a 120 Hz, ler novos
contatos. Não escrever transformações ou velocidades dos links.

1. **Observação.** Calcular COM e velocidade de COM por massa e offsets
   reais. Estimar cargas por pé a partir dos impulsos, com histerese e tempo
   mínimo para entrar/sair de apoio. Separar contatos externos, autocontato
   e deslizamento. Um contato de mão não vira automaticamente apoio de pé.
2. **Equilíbrio.** Avaliar COM e velocidade no plano dos apoios. Para um
   primeiro indicador no piso plano, usar `omega = sqrt(g / h)` e
   `xi = COM_horizontal + velocidade_horizontal / omega`, comparando-o com
   a área sustentada pelos contatos atuais. Desabilitar essa interpretação
   quando deitado, em voo ou com altura degenerada. Esse indicador serve à
   decisão de passo; não é força aplicada à raiz.
3. **Footwork.** Usar animações como forma nominal. Associar suas fases a
   pé esquerdo/direito em apoio ou balanço. Corrigir a passada livre conforme
   erro de velocidade e COM–apoio, com alcance, largura mínima e limites
   articulares. Antecipar/reter a troca de apoio conforme o touchdown físico,
   com timeout limitado para nunca travar a engine esperando um contato.
4. **Corpo e motores.** Distribuir a reação entre tornozelos, quadris e
   tronco, preservando continuidade de posição, velocidade e esforço.
   Calibrar frequência/amortecimento dos drives e torque disponível
   separadamente. O limite de esforço deve incluir gravidade, feedforward
   e servo; reportar saturação em vez de escondê-la.
5. **Assistência.** Continuar distinguindo auxílio de pose sem resultante,
   auxílio de velocidade XYZ e torque de postura. Todos aparecem no
   orçamento global. A recuperação pode ter orçamento temporariamente
   maior, autorizado pelo usuário, mas condicionado ao estado e aos apoios,
   com duração, força, trabalho e impulso registrados. Isso é uma concessão
   do jogo, não um resultado biomecânico dos artigos.

Para diagonais, alinhar as fases semânticas dos ciclos antes de misturar
frente/trás e lateral. Misturar timestamps iguais de clipes com touchdowns
em instantes diferentes pode colocar os dois pés em balanço ou cruzar as
pernas. Validar quatro direções cardinais antes das diagonais. Corrida requer
tratamento explícito da fase aérea; não herdar a hipótese de apoio contínuo
da caminhada.

## Recuperação e interação

Queda deve depender de orientação, altura relativa ao terreno, contatos e
persistência temporal. Um único frame inclinado não basta. Para frente/costas,
usar a direção anatômica anterior do tronco em relação à gravidade, conferida
no perfil; yaw da pelve projetado no chão é degenerado quando ela aponta para
cima ou para baixo. Queda lateral precisa de estado próprio/rolamento ou
espera controlada; não classificar arbitrariamente como frente.

Sequência proposta: queda → acomodação → seleção frente/costas → entrada
suave no clipe → execução com verificação dos apoios → estabilização em pé.
O tempo do clipe é uma referência ajustável: não declarar sucesso só porque
chegou ao último frame. Exigir tronco ereto, pés carregados e velocidade
moderada durante um intervalo. Falha por contato ausente, obstrução ou nova
queda leva a tentativa limitada/reavaliação, evitando um loop infinito de
forças crescentes. A rotação global da raiz do clipe não pode ser aplicada
como teleporte; deve resultar de contatos, articulações e auxílio declarado.

Durante a Physgun, manter impedância articular limitada e alvos corporais
suaves. Suspender a intenção de caminhar e impedir disputa entre propulsão
externa e ferramenta. A força de resposta deve ser visível ao usuário sem
tornar o boneco invulnerável ao arraste. Ao soltar, reavaliar apoios e postura
na localização presente e suavizar a transição. Suspender o corpo no ar não
pode manter sua altura por uma referência antiga.

## Critérios de aceitação por etapa

Os valores abaixo são propostas de ensaio inicial, não propriedades humanas
demonstradas. Publicar resultados medidos e cenários reprovados.

| Etapa | Ensaio | Evidência exigida |
| --- | --- | --- |
| Contrato | Transladar/rotacionar corpo e contexto; arrastar e soltar | Comandos equivalentes no referencial novo; nenhum retorno à posição anterior |
| Sensores | Pé no chão, pé na própria canela, mão no chão e sem chão | Apoios externos classificados corretamente; carga e deslizamento finitos |
| Músculos | Perturbação angular, postura sustentada, Physgun no tronco e na perna | Resistência limitada, amortecimento, ausência de NaNs e de esforço além do orçamento |
| Postura | 30 s parado; pequenos impulsos em quatro direções | Sem queda espontânea; transferência de carga observável; auxílio permanece secundário |
| Passo reativo | Impulsos crescentes | Passo modifica o apoio antes de cair; perturbações grandes ainda podem derrubar |
| Caminhada | Comando de 15 s, três orientações iniciais, depois parar | Anda fisicamente, alterna apoios, não patina sistematicamente, termina equilibrado sem dívida de percurso |
| Recuperação | Quedas frente/costas e leve variação das poses, pelo menos cinco de cada | Classificação correta; junta coesa; sucesso físico mantido após o clipe; falhas limitadas e diagnosticadas |
| Cobertura | Trás, laterais, diagonais, corrida, superfícies diferentes | Aprovação separada por comportamento, sem extrapolar o êxito da caminhada frontal |
| Desempenho | Vários ragdolls e perturbações simultâneas | Tempo de controle medido, trabalho por tick limitado, nenhuma busca ou espera sem limite |

Registrar altura/velocidade do COM, inclinação, cargas por apoio, fase nominal
e real, erro e saturação articular, deslizamento da sola, auxílio total e
tempo de CPU. Comparar com auxílio desligado para distinguir ganho de
controle de simples sustentação externa. Guardar as regressões de impacto,
queda livre, limites e ausência de ancoragem já existentes.

Aprovação automática não substitui inspeção visual de sola, joelho, quadril,
transições e reação à Physgun. Este estudo não demonstra capacidade em
escadas, plataformas móveis, quedas laterais arbitrárias, qualquer novo corpo
ou qualquer combinação de clipes. Essas capacidades precisam de ensaios e
controle adicionais; não são consequência automática de um bom retarget.
