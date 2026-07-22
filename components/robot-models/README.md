# Robot Models

Component of `blacknode-isaac`.

Node sources for this component belong in this folder. Until they move here,
nodes claim the component inline:

    @node(name="MyNode", component="robot-models", ...)

Once sources live here, declare the folder in `blacknode-package.toml`:

    [components.robot-models]
    nodes = ["components/robot-models/nodes"]

and the inline `component=` argument can be dropped — the loader infers it
from the directory.
