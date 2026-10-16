#!/usr/bin/env python3
"""O inventario das constantes de ajuste por ambiente (TEMPTUNE) da locomocao.

A reescrita da locomocao (vault 04_locomocao/reescrita_da_locomocao.md, R0)
troca estas constantes - lidas do ambiente uma vez, globais para todos os
atletas - por parametros tipados por modulo. Este inventario e o mapa: o nome,
a variavel, o valor, onde mora, o comentario que a explica e o grupo (pelo
prefixo). Tambem e o espaco de parametros que a calibracao (CMA-ES) e o RL vao
ajustar.

uso: tools/analysis/temptune_inventory.py [--saida docs/INVENTARIO_AJUSTES_LOCOMOCAO.md]
"""
import argparse
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
CALL = re.compile(r'tuneTemp[A-Za-z]*\(\s*"(T_[A-Za-z0-9]+)"\s*,\s*([^)]+?)\s*\)')
DECL = re.compile(r'(?:static\s+)?(?:const\s+)?(?:float|int|double|bool)\s+([A-Za-z_][A-Za-z0-9_]*)\s*=\s*tuneTemp')


def comment_before(lines, index):
    """O comentario logo acima (ou no fim) da linha da constante."""
    line = lines[index]
    tail = line.split('//', 1)
    if len(tail) > 1 and 'TEMPTUNE' not in tail[1]:
        return tail[1].strip()
    text = []
    j = index - 1
    while j >= 0 and lines[j].strip().startswith('//'):
        text.insert(0, lines[j].strip()[2:].strip())
        j -= 1
    return ' '.join(text)


def group(name):
    body = name[2:]
    match = re.match(r'[A-Z][a-z]+', body)
    return match.group(0) if match else body


def function_of(lines, index):
    """A funcao que contem a linha (o nome da definicao mais proxima acima)."""
    pattern = re.compile(r'^[A-Za-z].*\b([A-Za-z_][A-Za-z0-9_]*::[A-Za-z_~][A-Za-z0-9_]*)\s*\(')
    for j in range(index, -1, -1):
        match = pattern.match(lines[j])
        if match:
            return match.group(1)
    return 'arquivo'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--saida', default=str(ROOT / 'docs' / 'INVENTARIO_AJUSTES_LOCOMOCAO.md'))
    args = parser.parse_args()
    rows = []
    for path in sorted((ROOT / 'src').rglob('*.cpp')):
        lines = path.read_text(encoding='utf-8', errors='replace').splitlines()
        for index, line in enumerate(lines):
            for call in CALL.finditer(line):
                decl = DECL.search(line)
                note = comment_before(lines, index)
                rows.append({
                    'name': call.group(1),
                    'value': call.group(2).strip(),
                    'variable': decl.group(1) if decl else '',
                    'file': str(path.relative_to(ROOT)),
                    'line': index + 1,
                    'function': function_of(lines, index),
                    'group': group(call.group(1)),
                    'note': (note[:110] + '...') if len(note) > 110 else note,
                })
    names = {}
    for row in rows:
        names.setdefault(row['name'], []).append(row)
    out = []
    out.append('# Inventario das constantes de ajuste da locomocao (TEMPTUNE)\n')
    out.append('> Gerado por `tools/analysis/temptune_inventory.py` - nao editar a mao.\n')
    out.append('A reescrita da locomocao (vault `04_locomocao/reescrita_da_locomocao.md`, R0) troca estas '
               'constantes - lidas do ambiente uma vez e globais para todos os atletas - por parametros '
               'tipados por modulo, por atleta e normalizados pelo corpo. Este e o mapa (e o espaco de '
               'parametros da calibracao e do RL).\n')
    out.append(f'- **{len(names)} nomes** em **{len(rows)} leituras**.')
    by_file = {}
    for row in rows:
        by_file.setdefault(row['file'], []).append(row)
    for file, items in by_file.items():
        out.append(f'- `{file}`: {len({r["name"] for r in items})} nomes.')
    out.append('')
    groups = {}
    for row in rows:
        groups.setdefault(row['group'], []).append(row)
    out.append('## Os grupos (pelo prefixo do nome)\n')
    out.append('| Grupo | Nomes |')
    out.append('|---|---|')
    for name, items in sorted(groups.items(), key=lambda kv: -len(kv[1])):
        out.append(f'| {name} | {len({r["name"] for r in items})} |')
    out.append('')
    for file, items in by_file.items():
        out.append(f'## `{file}`\n')
        out.append('| Nome | Valor | Variavel | Linha | Funcao | O que faz (comentario) |')
        out.append('|---|---|---|---|---|---|')
        for row in items:
            note = row['note'].replace('|', '/')
            out.append(f'| `{row["name"]}` | {row["value"]} | {row["variable"] or "-"} | {row["line"]} | '
                       f'{row["function"]} | {note} |')
        out.append('')
    pathlib.Path(args.saida).write_text('\n'.join(out) + '\n', encoding='utf-8')
    print(f'{len(names)} nomes, {len(rows)} leituras -> {args.saida}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
