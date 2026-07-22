# Policy Runtime

Component of `blacknode-isaac`.

Node sources for this component belong in this folder. Until they move here,
nodes claim the component inline:

    @node(name="MyNode", component="policy-runtime", ...)

Once sources live here, declare the folder in `blacknode-package.toml`:

    [components.policy-runtime]
    nodes = ["components/policy-runtime/nodes"]

and the inline `component=` argument can be dropped — the loader infers it
from the directory.
