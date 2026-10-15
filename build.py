# Inlines sim/engine.js into web/ui-template.html -> web/index.html
e = open('sim/engine.js').read()
u = open('web/ui-template.html').read()
open('web/index.html', 'w').write(u.replace('/*ENGINE*/', e))
print('web/index.html rebuilt')
