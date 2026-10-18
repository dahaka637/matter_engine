"""matter-rl: a linha de comando do treino (o usuario e o Claude usam).

    matter-rl doctor                      confere GPU, modulo nativo e tarefas
    matter-rl iniciar [--run R] [--modo M] [--tarefa T]   abre o treino em segundo plano
    matter-rl servir  [...]               o mesmo, em primeiro plano (reabre na troca de modo)
    matter-rl treinar [...]               um processo de treino (o que o servir abre; a tarefa da
                                          locomocao, esquema matter-rl-locomocao-1, abre o treinador dela)
    matter-rl status | pausar | retomar | parar | salvar | avaliar
    matter-rl modo maximo|equilibrado|leve
    matter-rl tarefa <id>                 o treino aberto passa para outra tarefa (o mesmo cerebro)
    matter-rl ramo --de R --run N [--tarefa T]   um treino novo do ultimo checkpoint de outro (locomocao:
                                          o treino curto sozinho de uma novidade, com treinar --exercicio);
                                          --tarefa: o ramo leva a tarefa T dos assets, nao a copia do outro
    matter-rl renormalizar --run R --rodada N   a normalizacao refeita sem mudar a funcao (treino fechado)
    matter-rl recentrar --run R --rodada N      a media saturada de volta aos limites, a acao aplicada igual
    matter-rl peso <parcela> <valor> [--motivo M]   ("estilo" e o beta do juiz)
    matter-rl relatorio [--desde 1h]      o que mudou no periodo (Markdown)
    matter-rl graficos [--saida DIR]      PNG das curvas
    matter-rl folha --episodio ARQ [--cena NOME] [--saida PNG] [--de S --ate S]   um dump (previa_corpo.py)
    matter-rl afinidade treinar --politicas a,b   o juiz de avaliacao congelado (Corpo v2), uma vez
    matter-rl afinidade medir --politicas x       a afinidade de politicas com o acervo
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

from . import control, paths, report, storage, system

# O treino dos comandos sem --run.
DEFAULT_RUN = os.environ.get("MATTER_RL_RUN", "corpo")


def parse_duration(text: str) -> float:
    text = text.strip().lower()
    units = {"s": 1, "m": 60, "min": 60, "h": 3600, "d": 86400}
    for suffix in sorted(units, key=len, reverse=True):
        if text.endswith(suffix):
            return float(text[: -len(suffix)]) * units[suffix]
    return float(text) * 3600.0


def command_doctor(_: argparse.Namespace) -> int:
    import torch
    print(f"python {sys.version.split()[0]} · torch {torch.__version__} · CUDA {torch.version.cuda}")
    from .body_trainer import choose_device
    device = choose_device("auto")
    if device.type == "cuda":
        print(f"GPU ok: {torch.cuda.get_device_name(0)} (capacidade {torch.cuda.get_device_capability(0)})")
    else:
        print("GPU indisponivel: o treino usa a CPU")
    from . import native
    module = native.load()
    task = paths.task_path("aquisicao_p3a")
    batch = module.BodyBatch(str(task), environments=4, per_scene=4, threads=1)
    print(f"modulo nativo ok: {paths.NATIVE_DIR} · corpo {batch.body_id} ({batch.body_hash}) · "
          f"observacao {batch.obs_size}, critico {batch.critic_size}, acao {batch.action_size}")
    print(f"treinos em {paths.DATA}")
    inspector = paths.inspector()
    print(f"folhas de quadros: {inspector if inspector else 'MatterModelInspector ausente'}")
    temperature = system.cpu_temperature()
    if temperature:
        print(f"CPU {temperature:.0f} °C")
    return 0


def command_train(args: argparse.Namespace) -> int:
    from .locomotion_trainer import LocomotionTrainer, is_locomotion_run, is_locomotion_task
    # A locomocao (o RL dentro dos modulos do jogo, contrato v2): a tarefa do
    # esquema matter-rl-locomocao-1, ou um treino dela.
    if is_locomotion_run(args.run) or (args.tarefa and is_locomotion_task(paths.task_path(args.tarefa))):
        trainer = LocomotionTrainer(args.run, task=args.tarefa, mode=args.modo, environments=args.mundos,
                                    device=args.dispositivo, seed=args.semente, iterations=args.rodadas,
                                    drill=args.exercicio)
        return trainer.loop()
    from .body_trainer import BodyTrainer, is_body_run, is_body_task
    # O Corpo v2 (a tarefa do esquema 2, ou um treino dele) tem o treinador proprio.
    if is_body_run(args.run) or (args.tarefa and is_body_task(paths.task_path(args.tarefa))):
        trainer = BodyTrainer(args.run, task=args.tarefa, mode=args.modo, environments=args.mundos,
                              device=args.dispositivo, seed=args.semente, iterations=args.rodadas,
                              style=False if args.sem_estilo else None,
                              overlap=True if args.sobreposicao else (False if args.sem_sobreposicao else None),
                              transfer_from=args.de_checkpoint,
                              transfer=tuple(part for part in args.transferir.split(",") if part),
                              warmup=args.aquecimento, mirror_loss=args.espelho, command_gain=args.ganho_comando,
                              exploration=({"mode": "learned", "initial_rad": args.desvio_aprendido}
                                           if args.desvio_aprendido else None))
        return trainer.loop()
    print(f"o treino {args.run} nao e do Corpo (contrato 2) e a tarefa nao foi dada (--tarefa)", file=sys.stderr)
    return 1


def trainer_command(args: argparse.Namespace) -> list[str]:
    command = [sys.executable, "-m", "matter_rl", "treinar", "--run", args.run]
    if args.mundos:
        command += ["--mundos", str(args.mundos)]
    if args.dispositivo != "auto":
        command += ["--dispositivo", args.dispositivo]
    if getattr(args, "semente", None) is not None:
        command += ["--semente", str(args.semente)]
    if getattr(args, "sem_estilo", False):
        command += ["--sem-estilo"]
    if getattr(args, "sem_sobreposicao", False):
        command += ["--sem-sobreposicao"]
    if getattr(args, "sobreposicao", False):
        command += ["--sobreposicao"]
    if getattr(args, "de_checkpoint", None):
        command += ["--de-checkpoint", args.de_checkpoint, "--transferir", args.transferir]
    if getattr(args, "aquecimento", None) is not None:
        command += ["--aquecimento", str(args.aquecimento)]
    if getattr(args, "desvio_aprendido", None):
        command += ["--desvio-aprendido", str(args.desvio_aprendido)]
    if getattr(args, "espelho", None) is not None:
        command += ["--espelho", str(args.espelho)]
    if getattr(args, "ganho_comando", None) is not None:
        command += ["--ganho-comando", str(args.ganho_comando)]
    return command


def command_serve(args: argparse.Namespace) -> int:
    """Mantem o treino aberto: reabre na troca de modo e depois de um erro."""
    from .system import RESTART_CODE  # sem o PyTorch: o supervisor fica leve
    failures = 0
    first = True
    while True:
        command = trainer_command(args)
        # Tarefa e modo so na primeira abertura: depois valem os do run.json (a
        # troca de habilidade ou de modo reabre o treino com os novos).
        if first and args.tarefa:
            command += ["--tarefa", args.tarefa]
        if first and args.modo:
            command += ["--modo", args.modo]
        if first and getattr(args, "exercicio", None):
            command += ["--exercicio", args.exercicio]
        first = False
        started = time.time()
        code = subprocess.call(command, cwd=str(Path(__file__).resolve().parents[1]))
        if code == 0:
            return 0
        if code == RESTART_CODE:
            continue
        failures = failures + 1 if time.time() - started < 120 else 1
        if failures >= 3:
            print("o treino caiu 3 vezes seguidas; o supervisor desiste", file=sys.stderr)
            return code
        time.sleep(10)


def start_arguments(args: argparse.Namespace, command: list[str]) -> list[str]:
    """As opcoes de abertura de um treino (iniciar e servir): uma construcao so, para
    nenhuma se perder no caminho (a transferencia entre degraus, por exemplo)."""
    for flag, value in (("--tarefa", args.tarefa), ("--modo", args.modo), ("--mundos", args.mundos),
                        ("--semente", getattr(args, "semente", None)), ("--de-checkpoint", args.de_checkpoint),
                        ("--aquecimento", args.aquecimento), ("--desvio-aprendido", args.desvio_aprendido),
                        ("--espelho", args.espelho), ("--ganho-comando", args.ganho_comando),
                        ("--exercicio", getattr(args, "exercicio", None))):
        if value is not None and value != "":
            command += [flag, str(value)]
    if args.de_checkpoint:
        command += ["--transferir", args.transferir]
    for flag, value in (("--sem-estilo", args.sem_estilo), ("--sem-sobreposicao", args.sem_sobreposicao),
                        ("--sobreposicao", args.sobreposicao)):
        if value:
            command.append(flag)
    return command


def command_branch(args: argparse.Namespace) -> int:
    """Um ramo de um treino: o ultimo checkpoint, a configuracao e a copia da
    tarefa de outro treino num treino novo (o treino curto sozinho de uma
    novidade sem mexer no principal). O ramo comeca do mesmo cerebro e segue
    sozinho; o banco o registra quando ele abrir."""
    source = paths.run_dir(args.de)
    target = paths.run_dir(args.run)
    reference = storage.latest_checkpoint(source)
    if reference is None:
        print(f"o treino {args.de} nao tem checkpoint", file=sys.stderr)
        return 1
    if target.exists() and any(target.iterdir()):
        print(f"o treino {args.run} ja existe", file=sys.stderr)
        return 1
    task = paths.task_path(args.tarefa) if getattr(args, "tarefa", None) else None
    if task is not None and not task.exists():
        print(f"tarefa desconhecida: {args.tarefa}", file=sys.stderr)
        return 1
    target.mkdir(parents=True, exist_ok=True)
    import shutil
    for name in ("run.json", "tarefa.json"):
        if (source / name).exists():
            shutil.copy2(source / name, target / name)
    shutil.copytree(reference.path, target / reference.path.relative_to(source), dirs_exist_ok=True) \
        if reference.path.is_dir() else shutil.copy2(reference.path, target / reference.path.relative_to(source))
    config = json.loads((target / "run.json").read_text())
    config["end_reason"] = ""
    if task is not None:
        # A tarefa nova dos assets (o mesmo id que o outro tinha nao trocaria a copia).
        from .curriculum import carry_caps
        old = json.loads((source / "tarefa.json").read_text()) if (source / "tarefa.json").exists() else {}
        storage.write_json_atomic(target / "tarefa.json", carry_caps(old, json.loads(task.read_text())))
        config["task"] = args.tarefa
    storage.write_json_atomic(target / "run.json", config)
    print(f"ramo {args.run} de {args.de} (rodada {reference.iteration})")
    return 0


def command_start(args: argparse.Namespace) -> int:
    if control.ping():
        print("ja existe um treinador aberto (matter-rl status)")
        return 0
    directory = paths.run_dir(args.run)
    directory.mkdir(parents=True, exist_ok=True)
    log = open(directory / "treinador.log", "a")
    command = start_arguments(args, [sys.executable, "-m", "matter_rl", "servir", "--run", args.run])
    subprocess.Popen(command, cwd=str(Path(__file__).resolve().parents[1]), stdout=log, stderr=subprocess.STDOUT,
                     stdin=subprocess.DEVNULL, start_new_session=True)
    for _ in range(120):
        if control.ping():
            print(f"treino {args.run} aberto")
            return 0
        time.sleep(0.5)
    print(f"o treino nao respondeu em 60 s; veja {directory / 'treinador.log'}", file=sys.stderr)
    return 1


SERVICE = Path.home() / ".config" / "systemd" / "user" / "matter-rl.service"


def command_service(args: argparse.Namespace) -> int:
    """Retomar o treino sozinho ao ligar o PC (um servico de usuario do systemd)."""
    launcher = Path(__file__).resolve().parents[1] / "matter-rl"
    if args.acao == "instalar":
        SERVICE.parent.mkdir(parents=True, exist_ok=True)
        SERVICE.write_text(f"""[Unit]
Description=Treino de RL do Matter Engine (retoma ao ligar o PC)
After=graphical-session.target

[Service]
Type=simple
ExecStart={launcher} ao-ligar --run {args.run}
Nice=10

[Install]
WantedBy=default.target
""")
        subprocess.call(["systemctl", "--user", "daemon-reload"])
        return subprocess.call(["systemctl", "--user", "enable", "matter-rl.service"])
    if args.acao == "remover":
        subprocess.call(["systemctl", "--user", "disable", "matter-rl.service"])
        SERVICE.unlink(missing_ok=True)
        return subprocess.call(["systemctl", "--user", "daemon-reload"])
    enabled = subprocess.call(["systemctl", "--user", "is-enabled", "--quiet", "matter-rl.service"]) == 0
    print("retoma ao ligar o PC" if enabled else "nao retoma sozinho")
    return 0


def command_boot(args: argparse.Namespace) -> int:
    """O que o servico roda ao ligar o PC: so retoma se o treino nao foi parado
    pelo usuario (desligar o PC no meio do treino retoma; Parar nao)."""
    config_path = paths.run_dir(args.run) / "run.json"
    if not config_path.exists():
        return 0
    # O run.json lido direto (o RunConfig traria o PyTorch para o supervisor).
    config = json.loads(config_path.read_text())
    if config.get("end_reason") == "usuario":
        print("o treino foi parado pelo usuario: nao retomo")
        return 0
    if control.ping():
        return 0
    args.tarefa, args.modo, args.mundos, args.dispositivo = config.get("task"), config.get("mode"), None, "auto"
    return command_serve(args)


def command_status(args: argparse.Namespace) -> int:
    status = report.read_status(args.run)
    if status is None:
        print(f"nenhum treino '{args.run}' ainda")
        return 1
    if args.json:
        print(json.dumps(status, indent=2, ensure_ascii=False))
    else:
        print(report.format_status(status))
    return 0


def send(command: str, arguments: dict | None = None) -> int:
    answer = control.request(command, arguments)
    if answer is None:
        print("nenhum treinador aberto (matter-rl iniciar)")
        return 1
    if not answer.get("ok"):
        print(f"erro: {answer.get('error')}")
        return 1
    rest = {k: v for k, v in answer.items() if k not in ("ok", "protocol")}
    print(json.dumps(rest, ensure_ascii=False) if rest else "ok")
    return 0


def command_switch(args: argparse.Namespace) -> int:
    """Outra habilidade no mesmo cerebro: o treino aberto salva e fecha, e reabre
    com a tarefa nova (o que ele sabe continua). Fechar e reabrir vale para
    qualquer versao do treinador aberto."""
    if not paths.task_path(args.tarefa).exists():
        print(f"tarefa desconhecida: {args.tarefa}", file=sys.stderr)
        return 1
    status = report.read_status(args.run) or {}
    if control.ping():
        send("parar", {"autor": "usuario"})
        for _ in range(180):
            if not control.ping():
                break
            time.sleep(0.5)
        else:
            print("o treino nao fechou", file=sys.stderr)
            return 1
    args.modo, args.mundos = status.get("mode"), None
    # (As opcoes de abertura que o "tarefa" nao tem.)
    for name in ("semente", "de_checkpoint", "aquecimento", "desvio_aprendido", "espelho", "ganho_comando",
                 "exercicio", "sem_estilo", "sem_sobreposicao", "sobreposicao"):
        if not hasattr(args, name):
            setattr(args, name, None)
    args.transferir = getattr(args, "transferir", None) or "ator,normalizacao"
    live = paths.run_dir(args.run) / "tarefa.json"
    if live.exists():
        # A mesma tarefa com outro conteudo: a copia do treino passa a ser a nova.
        source = json.loads(paths.task_path(args.tarefa).read_text())
        if json.loads(live.read_text()).get("id") == source.get("id"):
            storage.write_json_atomic(live, source)
    return command_start(args)


def command_adjust(action: str, args: argparse.Namespace) -> int:
    """Um ajuste fora do laco (o treino fechado): a normalizacao ou o recentramento."""
    if control.ping():
        print("feche o treino antes (matter-rl parar)", file=sys.stderr)
        return 1
    from . import ajustes
    return ajustes.main([action, "--run", args.run, "--rodada", str(args.rodada), "--passos", str(args.passos)])


def command_report(args: argparse.Namespace) -> int:
    print(report.report(args.run, parse_duration(args.desde)))
    return 0


def command_plots(args: argparse.Namespace) -> int:
    output = Path(args.saida) if args.saida else paths.run_dir(args.run) / "graficos"
    for item in report.plots(args.run, output):
        print(item)
    return 0


def command_sheet(args: argparse.Namespace) -> int:
    dump = Path(args.episodio)
    if not dump.exists():
        print(f"dump ausente: {dump}", file=sys.stderr)
        return 1
    output = Path(args.saida) if args.saida else dump.with_suffix(".png")
    output.parent.mkdir(parents=True, exist_ok=True)
    report.sheet(dump, args.cena, output, args.quadros, args.vista, args.de, args.ate)
    print(output)
    return 0


def command_affinity(args: argparse.Namespace) -> int:
    """A afinidade com o acervo (o juiz de avaliacao congelado do Corpo v2)."""
    import numpy as np
    from . import native
    from .affinity import AffinityJudge, AffinitySettings, corrupted, describe, policy_pairs
    module = native.load()
    task = paths.task_path(args.tarefa)
    data = json.loads(task.read_text())
    body = str((task.parent / data["body"]).resolve())
    dataset = module.motion_dataset(body, str((task.parent / data["style"]["dataset"]).resolve()))
    judge_path = Path(args.juiz) if args.juiz else paths.DATA / "avaliacao" / "juiz_afinidade.pt"
    device = "cuda" if __import__("torch").cuda.is_available() else "cpu"
    if args.acao == "treinar":
        policies = [p for p in args.politicas.split(",") if p.strip()]
        rng = np.random.default_rng(0)
        negatives = [corrupted(np.asarray(dataset["pairs"]), rng)]
        for policy in policies:
            negatives.append(policy_pairs(module, str(task), policy))
        judge = AffinityJudge(dataset, AffinitySettings(), device)
        result = judge.train(np.concatenate(negatives))
        judge.save(judge_path, {"politicas": policies, "resultado": result})
        print(f"juiz de afinidade salvo em {judge_path}: {describe(result)}")
        return 0
    judge = AffinityJudge.load(judge_path, dataset, device)
    for policy in [p for p in args.politicas.split(",") if p.strip()]:
        print(f"{policy}: afinidade {judge.score(policy_pairs(module, str(task), policy)):.3f}")
    print(f"clipes reservados: {judge.score(judge.reserved_pairs):.3f}")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="matter-rl", description="Treino de RL do Matter Engine")
    sub = parser.add_subparsers(dest="command", required=True)

    def run_args(p, training=False):
        p.add_argument("--run", default=DEFAULT_RUN)
        if training:
            p.add_argument("--tarefa")
            p.add_argument("--modo", choices=list(system.MODES))
            p.add_argument("--mundos", type=int)
            p.add_argument("--dispositivo", default="auto", choices=["auto", "cuda", "cpu"])
            p.add_argument("--semente", type=int)
            p.add_argument("--rodadas", type=int)
            # A locomocao: todos os mundos num exercicio da tarefa (o treino curto
            # sozinho de uma novidade; fica no run.json do treino).
            p.add_argument("--exercicio")
            # O Corpo v2: sem o juiz de estilo (a variante B1; escolhida ao criar
            # o treino).
            p.add_argument("--sem-estilo", action="store_true")
            p.add_argument("--sem-sobreposicao", action="store_true")
            # A sobreposicao (a coleta da rodada seguinte enquanto a GPU atualiza) e
            # o padrao desde 16/10 (aprendizado equivalente, +41%; diario 28);
            # --sem-sobreposicao volta a serie.
            p.add_argument("--sobreposicao", action="store_true")
            # Um treino NOVO que nasce de outro (o ator e a normalizacao passam; o
            # critico e o juiz recomecam): "treino", "treino:rodada" ou uma pasta.
            p.add_argument("--de-checkpoint")
            p.add_argument("--transferir", default="ator,normalizacao")
            # Rodadas so do juiz e do critico antes de o ator voltar a aprender.
            p.add_argument("--aquecimento", type=int)
            # O desvio aprendido (o v1), com este desvio inicial em rad, em vez do fixo por grupo.
            p.add_argument("--desvio-aprendido", type=float)
            # O peso da perda do espelho no ator (0 = desligada; so ao criar o treino).
            p.add_argument("--espelho", type=float)
            # O ganho fixo do canal da velocidade pedida (padrao 3; 1 = o antigo).
            p.add_argument("--ganho-comando", type=float)
        return p

    sub.add_parser("doctor").set_defaults(func=command_doctor)
    affinity = sub.add_parser("afinidade", help="o juiz de avaliacao congelado do Corpo v2")
    affinity.add_argument("acao", choices=["treinar", "medir"])
    affinity.add_argument("--politicas", default="", help="politicas exportadas (.mrlpolicy), separadas por virgula")
    affinity.add_argument("--tarefa", default="corpo2")
    affinity.add_argument("--juiz", default="", help="o arquivo do juiz (padrao: <dados>/avaliacao/juiz_afinidade.pt)")
    affinity.set_defaults(func=command_affinity)
    branch = run_args(sub.add_parser("ramo", help="um treino novo a partir do ultimo checkpoint de outro (a locomocao)"))
    branch.add_argument("--de", required=True)
    branch.add_argument("--tarefa", help="a tarefa dos assets que o ramo leva (padrao: a copia do outro)")
    branch.set_defaults(func=command_branch)
    for name in ("renormalizar", "recentrar"):
        adjust = run_args(sub.add_parser(name))
        adjust.add_argument("--rodada", type=int, required=True, help="a rodada do checkpoint (o ultimo do treino)")
        adjust.add_argument("--passos", type=int, default=100)
        adjust.set_defaults(func=lambda a, n=name: command_adjust(n, a))
    run_args(sub.add_parser("treinar"), True).set_defaults(func=command_train)
    run_args(sub.add_parser("servir"), True).set_defaults(func=command_serve)
    run_args(sub.add_parser("iniciar"), True).set_defaults(func=command_start)
    status = run_args(sub.add_parser("status"))
    status.add_argument("--json", action="store_true")
    status.set_defaults(func=command_status)
    for name in ("pausar", "retomar", "parar", "salvar", "avaliar"):
        simple = sub.add_parser(name)
        simple.add_argument("--autor", default="usuario")
        simple.set_defaults(func=lambda a, n=name: send(n, {"autor": a.autor}))
    mode = sub.add_parser("modo")
    mode.add_argument("modo", choices=list(system.MODES))
    mode.add_argument("--autor", default="usuario")
    mode.set_defaults(func=lambda a: send("modo", {"modo": a.modo, "autor": a.autor}))
    switch = run_args(sub.add_parser("tarefa"))
    switch.add_argument("tarefa", help="o id da tarefa (assets/rl/tasks), ex.: aquisicao_p3b")
    switch.set_defaults(func=command_switch)
    weight = sub.add_parser("peso")
    weight.add_argument("componente")
    weight.add_argument("valor", type=float)
    weight.add_argument("--motivo", default="")
    weight.add_argument("--autor", default="usuario")
    weight.set_defaults(func=lambda a: send("peso", {"componente": a.componente, "valor": a.valor,
                                                     "motivo": a.motivo, "autor": a.autor}))
    rep = run_args(sub.add_parser("relatorio"))
    rep.add_argument("--desde", default="1h")
    rep.set_defaults(func=command_report)
    plot = run_args(sub.add_parser("graficos"))
    plot.add_argument("--saida")
    plot.set_defaults(func=command_plots)
    sheet = run_args(sub.add_parser("folha"))
    sheet.add_argument("--episodio", required=True, help="o dump (previa_corpo.py grava um por mundo)")
    sheet.add_argument("--cena")
    sheet.add_argument("--saida")
    sheet.add_argument("--quadros", type=int, default=8)
    sheet.add_argument("--vista")
    sheet.add_argument("--de", type=float, help="so o trecho a partir de (s)")
    sheet.add_argument("--ate", type=float, help="so o trecho ate (s)")
    sheet.set_defaults(func=command_sheet)
    service = run_args(sub.add_parser("servico"))
    service.add_argument("acao", choices=["instalar", "remover", "status"])
    service.set_defaults(func=command_service)
    run_args(sub.add_parser("ao-ligar")).set_defaults(func=command_boot)

    args = parser.parse_args(argv)
    return int(args.func(args) or 0)


if __name__ == "__main__":
    sys.exit(main())
