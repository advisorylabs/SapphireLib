#!/usr/bin/env node
/*
 * SapphireLib telemetry analyzer — build.js
 *
 * Bundles the analyzer into one self-contained HTML file, for handing to
 * someone as a single attachment or keeping on a laptop without the repo:
 *
 *   node tools/analyzer/build.js            → tools/analyzer/dist/sapphire-telemetry.html
 *   node tools/analyzer/build.js out.html   → out.html
 *
 * It inlines css/app.css and every js/*.js that index.html loads, in the same
 * order, and nothing else changes: the page behaves exactly like index.html.
 * No dependencies. dist/ is ignored by git; index.html stays the source.
 *
 * Team 96671H — Hitmen
 */
'use strict';

const fs = require('fs');
const path = require('path');

const root = __dirname;
const out = process.argv[2] ? path.resolve(process.argv[2]) : path.join(root, 'dist', 'sapphire-telemetry.html');

// A literal "</script" or "</style" inside inlined code would end the element
// early; "<\/" means the same thing to JavaScript and CSS never contains it.
const inlineSafe = (text, tag) => text.replace(new RegExp(`</${tag}`, 'gi'), `<\\/${tag}`);

let html = fs.readFileSync(path.join(root, 'index.html'), 'utf8');
let inlined = 0;

html = html.replace(/<link rel="stylesheet" href="(css\/[^"]+)">/g, (_, href) => {
  inlined++;
  const css = fs.readFileSync(path.join(root, href), 'utf8');
  return `<style>\n${inlineSafe(css, 'style')}</style>`;
});
html = html.replace(/<script src="(js\/[^"]+)"><\/script>/g, (_, src) => {
  inlined++;
  const js = fs.readFileSync(path.join(root, src), 'utf8');
  return `<script>\n${inlineSafe(js, 'script')}</script>`;
});

// Anything still pointing into the folder would break once the file moves.
const leftover = html.match(/(?:src|href)="(?:css|js)\/[^"]+"/g);
if (leftover) {
  console.error(`build.js: not inlined: ${leftover.join(', ')}`);
  process.exit(1);
}

fs.mkdirSync(path.dirname(out), { recursive: true });
fs.writeFileSync(out, html);
console.log(`${path.relative(process.cwd(), out)}: ${inlined} files inlined, ${(html.length / 1024).toFixed(0)} KB`);
