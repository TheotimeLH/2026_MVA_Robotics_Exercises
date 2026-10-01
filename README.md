# Robotic MVA 2025

This repository contains the exercices for the MVA robotics class, 2026.
Each notebook corresponds to one chapter of the class.
The notebooks are in Python and mostly based on [Pinocchio](https://github.com/stack-of-tasks/pinocchio).

## Getting started

### Clone this repository

Using Git via SSH:

```bash
git clone git@github.com:TheotimeLH/2026_MVA_Robotics_Exercises.git
```

Or via HTTPS:

```bash
git clone https://github.com/TheotimeLH/2026_MVA_Robotics_Exercises.git
```

### Install miniconda

- Linux: https://docs.conda.io/en/latest/miniconda.html
- macOS: https://docs.conda.io/en/latest/miniconda.html
- Windows: https://www.anaconda.com/download/

Only a little snippet is applied to your home .bashrc, everything else will be segmented!

### Run a notebook

- Go to your local copy of the repository.
- Open a terminal.
- Create the conda environment:

```bash
conda env create -f mva_robotics.yml
```

From there on, to work on a tutorial notebook, you only need to activate the environment:

```bash
conda activate mva_robotics_2026
```

Then launch the notebook with:

```bash
jupyter-lab
```

The notebook will be accessible from your web browser at [localhost:8888](http://localhost:8888).

Meshcat visualisation can be access in full page in `localhost:700N/static/` where N denotes the Nth meshcat instance created with the running kernel. You are also welcome to use VS Code with its Jupyter Notebook extension. 

## Updating the notebooks

Tutorials will be added one by one, for each new session you need to run `git pull`.
