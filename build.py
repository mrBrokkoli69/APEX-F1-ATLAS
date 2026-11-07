"""Собрать автономный HTML из src. Нужен только Python 3."""
from pathlib import Path
import json
root = Path(__file__).resolve().parents[1]
src = root / 'src'
code = "import * as THREE from 'three';\nimport { OrbitControls } from 'controls';\n"
code += 'const DATA=' + (src / 'catalog.json').read_text(encoding='utf-8') + ';\n'
code += 'const FIXTURES=' + json.dumps(json.loads((src / 'fixtures.json').read_text(encoding='utf-8'))) + ';\n'
for name in ['physics.js', 'model.js', 'app.js']:
    code += (src / name).read_text(encoding='utf-8') + '\n'
page = (src / 'layout.html').read_text(encoding='utf-8')
page = page.replace('/*STYLES*/', (src / 'styles.css').read_text(encoding='utf-8'))
page = page.replace('<!--IMPORTMAP-->', (src / 'vendor-importmap.html').read_text(encoding='utf-8'))
page = page.replace('/*APP*/', code)
(root / 'index.html').write_text(page, encoding='utf-8')
print('Built index.html')
