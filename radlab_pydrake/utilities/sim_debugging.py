def plot_diagram(name, plant=None, diagram=None):
    """
        given a finalized plant and/or a built diagram generate a png of the 
        urdf topology and connected diagram. The PNG will then save to the cwd and display in web browser

        @param name: filename to save [str]
        @param plant: finalized MultibodyPlant
        @param diagram: built diagram
    """
    import os
    os.environ['MPLBACKEND'] = 'Agg'  # noqa

    # Now that the environment is set up, it's safe to import matplotlib, etc.
    import matplotlib.pyplot as plt
    import webbrowser
    from pydrake.systems.drawing import plot_graphviz, plot_system_graphviz
    

    # If running under `bazel run`, output to cwd so the user can find it.
    # If running under `bazel test` avoid polluting the test's cwd.
    for env_name in ['BUILD_WORKING_DIRECTORY', 'TEST_TMPDIR']:
        if env_name in os.environ:
            os.chdir(os.environ[env_name])
    if plant:
        plt.figure(figsize=(11, 8.5), dpi=300)
        plot_graphviz(plant.GetTopologyGraphvizString())
        plt.savefig(f'{name}_topology.png')
        assert os.path.exists(f'{name}_topology.png')

    if diagram:
        plt.figure(figsize=(11, 8.5), dpi=300)
        plot_system_graphviz(diagram, max_depth=2)
        plt.savefig(f'{name}_diagram.png')
        assert os.path.exists(f'{name}_diagram.png')

    # Show the figures (but not when testing).
    if 'TEST_TMPDIR' not in os.environ:
        if plant:
            webbrowser.open_new_tab(url=f'{name}_topology.png')
        if diagram:
            webbrowser.open_new_tab(url=f'{name}_diagram.png')

