// Runs the JalMaa validation suite in Node: node tests/run-validation.js [seed]
const J = require('../sim/engine.js');
const seed = parseInt(process.argv[2] || '7', 10);
let fail = 0;
for (const T of J.TESTS) {
  const r = T.run(seed);
  const tag = r.pass === null ? 'REF ' : r.pass ? 'PASS' : 'FAIL';
  if (r.pass === false) fail++;
  console.log(`${tag}  ${T.name}\n      target: ${T.target}\n      result: ${r.value}`);
}
console.log(fail ? `\n${fail} test(s) failed` : '\nAll tests passed');
process.exit(fail ? 1 : 0);
