from pathlib import Path

def inplace_change(filename, old_string, new_string):
  with open(filename) as f:
    newText=f.read().replace(old_string, new_string)

  with open(filename, "w") as f:
    f.write(newText)

for filename in Path('../').glob('**/*.mk'):
    print(filename)
    inplace_change(filename,r'/home/seeger01/STM32Toolchain/projects/Met4FoF-SmartUpUnit', '..')

