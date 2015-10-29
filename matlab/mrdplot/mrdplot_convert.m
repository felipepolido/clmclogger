function [D,name,unit,freq] = mrdplot_convert(fname)
% [D,names,units,freq] = mrdplot_convert(fname)
% converts an MRDPLOT binary file into a Matlab matrix. If fname is
% given, the file is processed immediately. If no filename is given,
% a dialog box will ask to located the file
% The program returns the data matrix D, the names of the columns and
% their units, as well as the sampling frequency

% read in the file name
if ~exist('fname') | isempty(fname),
	[fname, pathname] = uigetfile('*','Select Data File');
	if (fname == 0),
		return;
	end;
	% concatenate pathname and filename and open file
	fname_store = fname;
	fname=strcat(pathname, fname);
end;

fid=fopen(fname, 'r','ieee-be');
if fid == -1,
	return;
end;

specs=fscanf(fid,'%d %d %d %f',4);  % [dummy,cols,rows,freq]
cols = specs(2);
rows = specs(3);
freq = specs(4);
t    = (1:rows)'/freq;  % the time column
fname= fname;
% read all variable names

for i=1:cols,
	name{i}=fscanf(fid,'%s',1);
	unit{i}=fscanf(fid,'%s',1);
end;
fscanf(fid,'%c',3); % there are three characters which must be skipped

% read the data
D = fread(fid, [cols,rows],'float32');
D=D';
fclose(fid);
