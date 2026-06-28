# CODING_RULES.md

## General

* Follow the existing coding style in the current file.
* Preserve consistency with surrounding code.
* Make the smallest possible change.
* Do not rewrite working code unless requested.
* Reuse existing functions before creating new ones.
* Keep code concise and readable.

---

## Naming

* Variables: snake_case
* Functions: snake_case
* Private methods: _snake_case
* Classes: PascalCase
* Constants: UPPER_CASE
* Preserve existing global names (e.g. `UAVs`).

---

## File Organization

Keep the existing section layout.

Use section separators when appropriate.

Example:

```python
# ---------------------------<UI Events>---------------------------
```

Do not reorganize files unless requested.

---

## Formatting

Prefer compact code.

Use horizontal space before vertical space.

Do not insert unnecessary blank lines.

Keep related statements together.

Only insert blank lines when separating different logical blocks.

Avoid formatting styles that unnecessarily increase file length.

---

## Line Wrapping

Prefer single-line statements whenever they remain readable.

Only wrap lines when:

* the line becomes too long (approximately 100–120 characters);
* readability clearly improves;
* nested logic requires it.

Do not split code simply because a formatter suggests it.

---

## Collections

Prefer compact formatting.

Good

```python
mapping = {"A": 0, "B": 1, "C": 2}
```

or

```python
mapping = {
    "A": 0, "B": 1, "C": 2,
    "D": 3, "E": 4,
}
```

Avoid

```python
mapping = {
    "A": 0,
    "B": 1,
    "C": 2,
    "D": 3,
    "E": 4,
}
```

unless each element is long or requires comments.

The same rule applies to lists, tuples and sets.

---

## Function Calls

Prefer

```python
result = calculate_path(start, goal, obstacles)
```

instead of

```python
result = calculate_path(
    start,
    goal,
    obstacles,
)
```

unless arguments are long or complex.

---

## Return Statements

Prefer

```python
return x, y, heading
```

instead of

```python
return (
    x,
    y,
    heading,
)
```

unless each returned value is a complex expression.

---

## Functions

* One responsibility per function.
* Prefer early return.
* Avoid unnecessary nesting.
* Keep existing function signatures whenever possible.

---

## Classes

* Keep responsibilities clear.
* Do not create unnecessary classes.
* Match the style already used in the project.

---

## Imports

Import order:

1. Standard library
2. Third-party libraries
3. Project modules

Remove unused imports.

Avoid wildcard imports.

---

## Type Hints

Use type hints whenever practical.

Preserve existing typing style.

---

## Async Programming

Use asyncio.

Never use time.sleep() inside async code.

Use:

* await
* asyncio.sleep()
* asyncio.create_task()

Do not block the event loop.

---

## Logging

Use the project's logger.

Prefer:

```python
logger.log(...)
```

Avoid:

```python
print(...)
```

Log unexpected errors with meaningful messages.

---

## Error Handling

Never use:

```python
except:
```

Catch specific exceptions.

Never silently ignore exceptions.

---

## Comments

Comment only when necessary.

Explain WHY.

Do not explain obvious code.

Keep comments concise.

---

## GUI Code

Keep UI code separate from business logic.

Avoid long-running tasks in the UI thread.

Use asynchronous tasks when appropriate.

---

## Refactoring

Do not rename APIs without request.

Do not change file structure.

Do not introduce unnecessary abstractions.

Avoid duplicated code.

---

## Before Finishing

Always verify:

* syntax
* imports
* naming consistency
* obvious runtime errors

Remove:

* unused imports
* dead code
* debug prints

---

## Never

* Change formatting style across the entire file.
* Reformat unrelated code.
* Introduce unnecessary line wrapping.
* Add excessive blank lines.
* Rewrite code only for stylistic reasons.

When editing existing code, preserve the surrounding formatting style.
## Existing Code Has Higher Priority

When editing a file:

- Match the formatting of nearby code.
- Do not introduce a different coding style.
- If the file already violates these rules, follow the existing file style instead of enforcing new formatting.