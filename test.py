import pint
ureg = pint.UnitRegistry()
print(3 * ureg.inch + 4 * ureg.cm)