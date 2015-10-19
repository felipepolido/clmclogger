function [D,names,units,freq] = mrdplot_convert(fname)
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
names=zeros(cols,40);
units=zeros(cols,40);
% read all variable names

for i=1:cols,
	temp=fscanf(fid,'%s',1);
	names(i,:)=[temp zeros(1,40-length(temp))];
	temp=fscanf(fid,'%s',1);
	units(i,:)=[temp zeros(1,40-length(temp))];
end;
fscanf(fid,'%c',3); % there are three characters which must be skipped

% read the data
D = fread(fid, [cols,rows],'float32');
D=D';
fclose(fid);
