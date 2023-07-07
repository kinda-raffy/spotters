# Spotters 🐶

Welcome to Spotters! Documentation is available under the wiki tab of this repository, and guidelines for contributing to this project are included below. This is still a major work in progress, so parts of the application and the corresponding documentation are subject to significant change moving forward.

## Guidelines

- Do not push to main.
- Categorise branches via `/` (`nav/localisation`, `nav/init_graph`)
- Submodules have been defaulted to `dev` in their respective repositories. To update dependencies in `spotters`, `git submodule update --remote` will replace the current commit with the latest on `dev`.
- To work on submodules, create an approapiate branch and merge with `dev` to publish changes. 
