# Workflows

Please mind that some of the workflows are run on a self-hosted runner.

## Setup Self-Hosted Runner


To set up a self-hosted runner, follow the instructions in the GitHub documentation linked [here](https://docs.github.com/en/actions/hosting-your-own-runners/managing-self-hosted-runners/adding-self-hosted-runners). 

After setting up, you can configure your workflows to use the self-hosted runner by specifying `runs-on: self-hosted` in your workflow YAML files.

At the time of writing this, we used a Jetson Nano as self-hosted runner.