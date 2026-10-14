"""matter-rl: a linha de comando do treino (o usuario e o Claude usam).

    matter-rl doctor                      confere GPU, modulo nativo e tarefas
    matter-rl iniciar [--run R] [--modo M] [--tarefa T]   abre o treino em segundo plano
    matter-rl voltar --checkpoint N [--run R]   o treino (fechado) volta para um checkpoint, sem apagar nada
    matter-rl servir  [...]               o mesmo, em primeiro plano (reabre na troca de modo)
    matter-rl treinar [...]               um processo de treino (o que o servir abre)
    matter-rl status | pausar | retomar | parar | salvar | avaliar
    matter-rl modo maximo|equilibrado|leve
    matter-rl tarefa <id>                 o treino aberto passa para outra habilidade (o mesmo cerebro)
    matter-rl explorar <desvio>           reabre a exploracao (o desvio da acao volta a pelo menos esse)
    matter-rl professor [--minutos N]     aprende com a locomocao fisica do jogo (imitando) e volta ao treino normal
    matter-rl peso <parcela> <valor> [--motivo M]   (no Corpo v2: "estilo" e o beta do juiz)
    matter-rl relatorio [--desde 1h]      o que mudou no periodo (Markdown)
    matter-rl graficos [--saida DIR]      PNG das curvas
    matter-rl folha [--episodio ARQ] [--cena NOME] [--saida PNG] [--de S --ate S]
    matter-rl folha --checkpoint X [--mundo N] [--solver jogo|treino]   um mundo de um checkpoint
    matter-rl folha --professor [--de S --ate S]   um episodio do professor (a locomocao fisica do jogo)
    matter-rl folha --referencia [--de S --ate S]  a referencia da imitacao (o clipe no corpo do RL)
    matter-rl comparar A B                a bateria em dois checkpoints, lado a lado
    matter-rl episodios                   as gravacoes da ultima avaliacao
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

DEFAULT_RUN = "principal"


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
    from .trainer import choose_device
    device = choose_device("auto")
    if device.type == "cuda":
        print(f"GPU ok: {torch.cuda.get_device_name(0)} (capacidade {torch.cuda.get_device_capability(0)})")
    else:
        print("GPU indisponivel: o treino usa a CPU")
    from . import native
    module = native.load()
    task = paths.task_path("h1_ficar_em_pe")
    batch = module.Batch(str(task), environments=4, threads=1)
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
    from .body_trainer import BodyTrainer, is_body_run, is_body_task
    # O Corpo v2 (a tarefa do esquema 2, ou um treino dele) tem o treinador proprio.
    if is_body_run(args.run) or (args.tarefa and is_body_task(paths.task_path(args.tarefa))):
        trainer = BodyTrainer(args.run, task=args.tarefa, mode=args.modo, environments=args.mundos,
                              device=args.dispositivo, seed=args.semente, iterations=args.rodadas,
                              style=False if args.sem_estilo else None,
                              overlap=False if args.sem_sobreposicao else None)
        return trainer.loop()
    from .trainer import Trainer
    trainer = Trainer(args.run, task=args.tarefa, mode=args.modo, environments=args.mundos, device=args.dispositivo,
                      seed=args.semente, iterations=args.rodadas)
    return trainer.loop()


def trainer_command(args: argparse.Namespace) -> list[str]:
    command = [sys.executable, "-m", "matter_rl", "treinar", "--run", args.run]
    if args.mundos:
        command += ["--mundos", str(args.mundos)]
    if args.dispositivo != "auto":
        command += ["--dispositivo", args.dispositivo]
    if getattr(args, "sem_estilo", False):
        command += ["--sem-estilo"]
    if getattr(args, "sem_sobreposicao", False):
        command += ["--sem-sobreposicao"]
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


def command_start(args: argparse.Namespace) -> int:
    if control.ping():
        print("ja existe um treinador aberto (matter-rl status)")
        return 0
    directory = paths.run_dir(args.run)
    directory.mkdir(parents=True, exist_ok=True)
    log = open(directory / "treinador.log", "a")
    command = [sys.executable, "-m", "matter_rl", "servir", "--run", args.run]
    for flag, value in (("--tarefa", args.tarefa), ("--modo", args.modo), ("--mundos", args.mundos)):
        if value:
            command += [flag, str(value)]
    for flag, value in (("--sem-estilo", args.sem_estilo), ("--sem-sobreposicao", args.sem_sobreposicao)):
        if value:
            command.append(flag)
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


def command_back(args: argparse.Namespace) -> int:
    """Volta o treino para um checkpoint, com o treinador fechado. Nada se apaga:
    o escolhido ganha uma copia em protegidos, e os mais novos vao para la
    (assim nao colidem com os que o treino vai gravar). latest.json aponta para
    ele; a copia da tarefa e o run.json voltam para a tarefa dele (uma copia de
    outra tarefa faria o treinador zerar a saida da rede ao reabrir); e as
    politicas exportadas passam a ser as dele."""
    import shutil
    if control.ping():
        print("feche o treino antes (matter-rl parar)", file=sys.stderr)
        return 1
    directory = paths.run_dir(args.run)
    refs = storage.list_checkpoints(directory)
    chosen = next((r for r in refs if r.iteration == int(args.checkpoint)), None)
    if chosen is None:
        print(f"checkpoint {args.checkpoint} nao existe em {directory / 'checkpoints'}", file=sys.stderr)
        return 1
    protected = paths.DATA / "protegidos"
    protected.mkdir(parents=True, exist_ok=True)
    copy = protected / f"{args.run}_voltar_{chosen.path.name}"
    if not copy.exists():
        shutil.copytree(chosen.path, copy)
    moved = []
    for ref in refs:
        if ref.iteration > chosen.iteration:
            target = protected / f"{args.run}_depois_de_{chosen.iteration}_{ref.path.name}"
            if target.exists():
                print(f"ja existe {target}; nada movido", file=sys.stderr)
                return 1
            shutil.move(str(ref.path), str(target))
            moved.append(target.name)
    storage.write_json_atomic(directory / "latest.json", {"iteration": chosen.iteration, "path": chosen.path.name})
    # A copia da tarefa e a do checkpoint (os caminhos absolutos, como o treinador faz).
    task = chosen.manifest.get("task")
    if task and paths.task_path(task).exists():
        source = paths.task_path(task)
        data = json.loads(source.read_text())
        data["body"] = str((source.parent / data["body"]).resolve())
        for clip in (data.get("imitation") or {}).get("clips", []):
            clip["clip"] = str((source.parent / clip["clip"]).resolve())
        storage.write_json_atomic(directory / "tarefa.json", data)
        config = json.loads((directory / "run.json").read_text())
        config["task"] = task
        storage.write_json_atomic(directory / "run.json", config)
    # As politicas exportadas: as do checkpoint, com os nomes das entradas.
    from .model import ActorCritic, export_policy
    state = storage.load_checkpoint(chosen)
    manifest = chosen.manifest
    model = ActorCritic(len(manifest["obs_names"]), len(manifest["critic_names"]), len(manifest["action_names"]))
    model.load_state_dict(state["model"])
    meta = {"observationLayout": "matter.rl.observation", "observationVersion": 1, "actionLayout": "matter.rl.action",
            "bodyId": manifest.get("body_id"), "bodyHash": manifest.get("body_hash"), "taskId": task, "run": args.run,
            "iteration": chosen.iteration, "steps": manifest.get("steps", 0),
            "observationNames": list(manifest["obs_names"])}
    for name in ("policy_latest", "policy_best"):
        export_policy(model, directory / "exports" / f"{name}.mrlpolicy", meta)
    db = storage.Database()
    db.event(args.run, "voltar", f"de volta ao checkpoint {chosen.iteration} ({task}); mais novos em protegidos: "
             + (", ".join(moved) or "nenhum"), args.autor)
    print(f"treino {args.run} de volta ao {chosen.path.name} ({task}); mais novos em {protected}: {', '.join(moved) or 'nenhum'}")
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
    return command_start(args)


def command_report(args: argparse.Namespace) -> int:
    print(report.report(args.run, parse_duration(args.desde)))
    return 0


def command_plots(args: argparse.Namespace) -> int:
    output = Path(args.saida) if args.saida else paths.run_dir(args.run) / "graficos"
    for item in report.plots(args.run, output):
        print(item)
    return 0


def latest_dump(run: str) -> Path | None:
    dumps = sorted((paths.run_dir(run) / "episodes").glob("aval-*.dump"))
    return dumps[-1] if dumps else None


def reference_dump(task: Path, seconds: float, record: Path) -> Path:
    """A referencia da imitacao da tarefa (os clipes no corpo do RL, a pelve
    andando com a velocidade dela), gravada para a folha: o que a politica
    deveria fazer em cada instante."""
    import json
    from . import native
    data = json.loads(Path(task).read_text())
    imitation = data.get("imitation") or {}
    if not imitation.get("clips"):
        raise SystemExit("a tarefa nao tem imitacao (secao imitation)")
    clips = [str((Path(task).parent / clip["clip"]).resolve()) for clip in imitation["clips"]]
    body = str((Path(task).parent / data["body"]).resolve())
    record.parent.mkdir(parents=True, exist_ok=True)
    record.write_text(native.load().reference_dump(body, clips, seconds, "referencia"))
    return record


def teacher_dump(task: str, seconds: float, seed: int, record: Path) -> Path:
    """Um episodio do professor (a locomocao fisica do jogo conduzindo o corpo
    do RL, no roteiro do aprendizado guiado), gravado para a folha."""
    from . import native
    import numpy as np
    batch = native.load().Batch(str(paths.task_path(task)), environments=1, per_scene=1, threads=1, seed=seed,
                                evaluation=True, evaluation_seconds=seconds + 1.0, solver_position=20,
                                solver_velocity=20, posture="alerta", teacher_assets=str(paths.ASSETS))
    zero = np.zeros((1, batch.action_size), dtype=np.float32)
    record.parent.mkdir(parents=True, exist_ok=True)
    with open(record, "w") as out:
        out.write("S professor\n")
        for step in range(int(seconds / batch.policy_dt)):
            batch.step(zero)
            if step % 2 == 1:
                out.write(f"F {(step + 1) * batch.policy_dt:.4f} 0\n")
                out.write(batch.frame_lines(0))
    return record


def command_sheet(args: argparse.Namespace) -> int:
    scenario = args.cena
    if args.referencia:
        task = paths.task_path(args.tarefa) if args.tarefa else paths.run_dir(args.run) / "tarefa.json"
        dump = reference_dump(task, (args.ate or 3.0) + 0.1, paths.run_dir(args.run) / "folhas" / "referencia.dump")
        output = Path(args.saida) if args.saida else dump.with_suffix(".png")
        report.sheet(dump, "referencia", output, args.quadros, args.vista, args.de, args.ate)
        print(output)
        return 0
    if args.professor:
        dump = teacher_dump(args.tarefa or "h3_andar", args.ate or 20.0, args.semente,
                            paths.run_dir(args.run) / "folhas" / "professor.dump")
        scenario = "professor"
        output = Path(args.saida) if args.saida else dump.with_suffix(".png")
        report.sheet(dump, scenario, output, args.quadros, args.vista, args.de, args.ate)
        print(output)
        return 0
    if args.checkpoint:
        # Roda a bateria nesse checkpoint (fora do treino) gravando o mundo pedido.
        from . import compare
        reference = compare.resolve(args.run, args.checkpoint)
        record = paths.run_dir(args.run) / "folhas" / f"ck-{reference.iteration:09d}-mundo{args.mundo}"
        from .evaluation import Evaluator, battery_name, task_batteries
        name = args.bateria or battery_name(args.postura)
        spec = next((b for b in task_batteries(paths.run_dir(args.run) / "tarefa.json") if b["name"] == name), None)
        if spec is None:
            print(f"a tarefa do treino nao tem a bateria {name}", file=sys.stderr)
            return 1
        compare.evaluate(args.run, args.checkpoint, record=record, watch=(args.mundo,), solvers=[args.solver],
                         names=[name])
        label, scene = Evaluator.labels(spec, args.solver)
        dump = record.with_name(f"{record.stem}-{label}.dump")
        scenario = scenario or f"{scene}_mundo{args.mundo}_{args.solver}"
    else:
        dump = Path(args.episodio) if args.episodio else latest_dump(args.run)
    if dump is None:
        print("nenhum episodio gravado ainda (as avaliacoes gravam)")
        return 1
    output = Path(args.saida) if args.saida else paths.run_dir(args.run) / "folhas" / f"{dump.stem}.png"
    output.parent.mkdir(parents=True, exist_ok=True)
    report.sheet(dump, scenario, output, args.quadros, args.vista, args.de, args.ate)
    print(output)
    return 0


def command_compare(args: argparse.Namespace) -> int:
    from . import compare
    print(compare.compare(args.run, args.a, args.b, args.threads))
    return 0


def command_episodes(args: argparse.Namespace) -> int:
    dumps = sorted((paths.run_dir(args.run) / "episodes").glob("aval-*.dump"))
    for dump in dumps[-6:]:
        print(dump)
        for name in report.scenarios(dump):
            print(f"  {name}")
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
            # O Corpo v2: sem o juiz de estilo (a variante B1; escolhida ao criar
            # o treino) e sem a atualizacao sobreposta a coleta.
            p.add_argument("--sem-estilo", action="store_true")
            p.add_argument("--sem-sobreposicao", action="store_true")
        return p

    sub.add_parser("doctor").set_defaults(func=command_doctor)
    affinity = sub.add_parser("afinidade", help="o juiz de avaliacao congelado do Corpo v2")
    affinity.add_argument("acao", choices=["treinar", "medir"])
    affinity.add_argument("--politicas", default="", help="politicas exportadas (.mrlpolicy), separadas por virgula")
    affinity.add_argument("--tarefa", default="corpo2")
    affinity.add_argument("--juiz", default="", help="o arquivo do juiz (padrao: <dados>/avaliacao/juiz_afinidade.pt)")
    affinity.set_defaults(func=command_affinity)
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
    explore = sub.add_parser("explorar")
    explore.add_argument("desvio", type=float, help="o desvio minimo da acao (unidades de acao; o inicio e ~0,6)")
    explore.add_argument("--autor", default="usuario")
    explore.add_argument("--motivo", default="")
    explore.set_defaults(func=lambda a: send("explorar", {"desvio": a.desvio, "autor": a.autor, "motivo": a.motivo}))
    teacher = sub.add_parser("professor")
    teacher.add_argument("--minutos", type=float, default=None,
                         help="quanto tempo a politica imita o professor (padrao: o da tarefa, ou 15)")
    teacher.add_argument("--autor", default="usuario")
    teacher.set_defaults(func=lambda a: send("professor", {"autor": a.autor,
                                                           **({"minutos": a.minutos} if a.minutos else {})}))
    back = run_args(sub.add_parser("voltar"))
    back.add_argument("--checkpoint", required=True, help="a rodada do checkpoint (ex.: 59316)")
    back.add_argument("--autor", default="usuario")
    back.set_defaults(func=command_back)
    switch = run_args(sub.add_parser("tarefa"))
    switch.add_argument("tarefa", help="o id da tarefa (assets/rl/tasks), ex.: h1b_em_pe_posturado")
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
    sheet.add_argument("--episodio")
    sheet.add_argument("--cena")
    sheet.add_argument("--saida")
    sheet.add_argument("--quadros", type=int, default=8)
    sheet.add_argument("--vista")
    sheet.add_argument("--checkpoint", help="rodada, ultimo ou melhor: avalia fora do treino e grava o mundo")
    sheet.add_argument("--mundo", type=int, default=0)
    sheet.add_argument("--solver", default="jogo", choices=["jogo", "treino"])
    sheet.add_argument("--postura", default="natural", help="a postura pedida (natural, alerta)")
    sheet.add_argument("--bateria", help="a bateria pelo nome (andar_frente_v1, ir_ate_10m_v1...); sem ela, a de pe da postura")
    sheet.add_argument("--de", type=float, help="so o trecho a partir de (s)")
    sheet.add_argument("--ate", type=float, help="so o trecho ate (s)")
    sheet.add_argument("--professor", action="store_true",
                       help="um episodio do professor (a locomocao fisica do jogo no corpo do RL)")
    sheet.add_argument("--referencia", action="store_true",
                       help="a referencia da imitacao (o clipe no corpo do RL) nos mesmos instantes da folha")
    sheet.add_argument("--tarefa", help="com --professor ou --referencia: a tarefa (padrao h3_andar / a do treino)")
    sheet.add_argument("--semente", type=int, default=11, help="com --professor: o sorteio do roteiro")
    sheet.set_defaults(func=command_sheet)
    comparison = run_args(sub.add_parser("comparar"))
    comparison.add_argument("a", help="rodada, ultimo ou melhor")
    comparison.add_argument("b", help="rodada, ultimo ou melhor")
    comparison.add_argument("--threads", type=int, default=2)
    comparison.set_defaults(func=command_compare)
    run_args(sub.add_parser("episodios")).set_defaults(func=command_episodes)
    service = run_args(sub.add_parser("servico"))
    service.add_argument("acao", choices=["instalar", "remover", "status"])
    service.set_defaults(func=command_service)
    run_args(sub.add_parser("ao-ligar")).set_defaults(func=command_boot)

    args = parser.parse_args(argv)
    return int(args.func(args) or 0)


if __name__ == "__main__":
    sys.exit(main())
