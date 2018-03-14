# Route Finding Search Visualisation

This provides a visualization of various search algorithm on the manhattan and CPH map from the DTU course 08180 based on a fork of [aima-python](https://github.com/aimacode/aima-python).

The modifies files are:

* `gui/map_parser.py` containing a parser for the provided map format.
* `gui/map_problem` that opens the visualization in a GUI.

To run:
```
python gui/map_problem.py data/cph.txt
python gui/map_problem.py data/manhattan.txt
```

## Python 3.4 and up

This code requires Python 3.4 or later, and does not run in Python 2. You can [install Python](https://www.python.org/downloads) or use a browser-based Python interpreter such as [repl.it](https://repl.it/languages/python3).
You can run the code in an IDE, or from the command line with `python -i filename.py` where the `-i` option puts you in an interactive loop where you can run Python functions. See [jupyter.org](http://jupyter.org/) for instructions on setting up your own Jupyter notebook environment, or run the notebooks online with [try.jupiter.org](https://try.jupyter.org/). 

