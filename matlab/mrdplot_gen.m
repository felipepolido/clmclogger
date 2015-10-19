function mrdplot_gen(data,names,units,freq,fname)
% mrdplot_gen(data,names,units,freq)
% writes a data matrix into MRDPLOT binary file. The names 
% matrix contains the variable names, units contains the units
% and freq the sampling frequency.

% read in the file name

if ~exist('fname') | isempty(fname),
	[fname, pathname] = uiputfile('d*','Save under which filename?');
	if (fname == 0),
		return;
	end;

	% concatenate pathname and filename and open file
	fname_store = fname;
	fname=strcat(pathname, fname);
end

% open the file
fid=fopen(fname, 'w','ieee-be');
if fid == -1,
  return;
end;

% write the variables

[rows,cols]=size(data);
fprintf(fid,'%d %d %d %d',cols*rows,cols,rows,freq);

% write all variable names

for i=1:cols,
	temp=names(i,:);
	ind = find(temp==0 | temp==' ');
	temp(ind)=[];
	fprintf(fid,'%s  ',temp);
	temp=units(i,:);
	ind = find(temp==0 | temp==' ');
	temp(ind)=[];
	fprintf(fid,'%s  ',temp);
end;
fprintf(fid,'\n');

% write the data
data = fwrite(fid,data','float32');
fclose(fid);
