import glob
import re

files = glob.glob('src/**/*.ts', recursive=True)
imports = set()
for f in files:
    try:
        content = open(f, encoding='utf-8').read()
        # Find imports of the form: import ... from "./libs/..."
        found = re.findall(r'from\s+[\'"]\./libs/([^\'"]+)[\'"]', content)
        for name in found:
            imports.add(name.replace('.js', '').replace('.ts', ''))
    except Exception as e:
        print(f"Error reading {f}: {e}")

print("Imported libs:", sorted(list(imports)))
