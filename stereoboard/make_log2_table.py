import numpy as np;
from matplotlib import pyplot as pl;

def make_log2_table(resolution=100):
	x = range(resolution);
	inp = np.asarray(x) / float(resolution);
	y = np.round(resolution * (inp * np.log2(inp)));
	y[0] = 0;
	
	pl.figure();
	pl.plot(x,y);
	pl.show();
	pl.title('p vs. y = p log2(p) at given resolution');
	
	fname = 'log2_table_' + str(resolution) + '.txt';
	#np.savetxt(fname, y);
	f = open(fname, 'w');
	f.write('static const int8_t log2_lookup_table[' + str(resolution)  +  '] = {\n');
	for i in x:
		if(i < resolution - 1):
			f.write(str(int(-y[i])) + ', ');
		else:
			f.write(str(int(-y[i])) + '\n};\n');
	f.close();
