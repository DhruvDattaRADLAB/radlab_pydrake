Ok how to update package: 

```sh
rm -rf dist 
# * CHANGE THE VERSION in setup.py 

python3 -m build 
```

# To install locally cd into the `dist/` folder and use 
```sh
pip install --upgrade *.whl
```

# to push to the pypl library
twine upload dist/*

pip install -e . 

# THEN 
# When trying to use RADLAB roboball simulation 
pip install radlab_pydrake --target=install/simulator/lib/python3.12/site-packages 
```




 

 