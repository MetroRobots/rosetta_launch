#!/usr/bin/python3
import pathlib
import re

SOURCE_PATTERN = re.compile(
    r'('
    r'\[source\]\(([^)]+)\)\n'    # source link
    r'('
    r'```(\w*)\n'  # First line of source
    r'([^`]*)'        # Contents
    r'\n```'
    r'|[^`]'
    r'))', re.DOTALL)

fn = 'README.md'
s = open(fn).read()
for whole_match, filepath, source_block, source_type, contents in SOURCE_PATTERN.findall(s):
    if not pathlib.Path(filepath).exists():
        continue

    if len(source_block) == 1:
        trail = source_block
    else:
        trail = ''
    contents = open(filepath).read().rstrip()
    if '<launch>' in contents:
        source_type = 'xml'
    else:
        source_type = 'python'
    new_match = f'[source]({filepath})\n```{source_type}\n{contents}\n```{trail}'
    s = s.replace(whole_match, new_match)

with open(fn, 'w') as f:
    f.write(s)
