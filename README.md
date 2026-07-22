
<img width="700" height="525" alt="matter_engine_logo" src="https://github.com/user-attachments/assets/eabc1a75-b191-463f-b542-f15dd1f11165" />

# MatterEngine

Motor 3D experimental em C++20, desenvolvido do zero com foco em física
precisa, renderização moderna, arquitetura modular e alto desempenho.

## Estado atual

- Vulkan 1.4 com Volk, VMA, HDR, TAA e Cascaded Shadow Maps;
- NVIDIA PhysX 5.9 com simulação fixa a 120 Hz;
- job system multithread com workers persistentes e work stealing;
- importação glTF 2.0, materiais físicos e áudio de impacto;
- Capsule Controller, props dinâmicos e Physgun;
- Workbench em Dear ImGui com laboratório e visualizador de objetos.

O projeto ainda está em desenvolvimento e não é considerado pronto para
produção.

## Compilação

Requisitos: Windows 10/11 x64, Visual Studio 2022 com C++, CMake 3.20+ e
driver compatível com Vulkan 1.4.

```powershell
.\tools\build.cmd -Configuration Debug
.\tools\run.cmd -Configuration Debug
```

Verificação completa:

```powershell
.\tools\check-architecture.cmd
.\tools\build.cmd -Configuration Debug
ctest --test-dir build-modern -C Debug --output-on-failure
```

## Estrutura

- `src/Engine`: runtime, física, áudio, geometria, materiais e renderização;
- `src/Workbench`: aplicação e ferramentas de desenvolvimento;
- `assets`: modelos, sons, fontes, shaders e identidade visual;
- `tests`: testes automatizados da fundação;
- `tools`: scripts de build, validação e preparação de assets.

Mais detalhes estão em [Arquitetura](docs/ARCHITECTURE.md),
[Roadmap](docs/ROADMAP.md) e
[Autoria de assets físicos](docs/PHYSICAL_ASSET_AUTHORING.md).

<img width="400" height="200" alt="image" src="https://github.com/user-attachments/assets/d305083f-a401-4a46-b0ce-09df6d6b3d19" />
