function mrdplot_functions(task)
% this part of the program implements the switch to various functiontalities
% of MRDPLOT
% Copyright 1998 Michael Rochon-Duck & Stefan Schaal
% Computational Learning and Motor Control Laboratory
% University of Southern California
% December 1997

global MRDS;  % the structure containing all the data for MRDPLOT

switch task,
case 'init',
	init_mrdplot;
case 'open',
	mrdplot_open(0);
case 'fast_open',
	mrdplot_open(1);
case 'zoom',
	mrdplot_zoom;
case 'cursor',
	mrdplot_cursor;
case 'clear_all',
	clear_all;
case 'key_press',
	key_press;
case 'varlist',
	mrdplot_varlist;
case 'add_subplot',
	add_subplot;
case 'delete_subplot',
	delete_subplot;
otherwise,
	return;
end;
%----------------------------------------------------------------------------------------------
% the function that initalizes MRDPLOT
function init_mrdplot()

global MRDS;  % the structure containing all the data for MRDPLOT
% all pointers are stored in the MRDS structure, which is global and will thus
% not be forgotten

% is MRDPLOT already running? Check whether the windows exist.
if ~isempty(MRDS) & ishandle(MRDS.cptr) & ishandle(MRDS.dptr),
	disp('MRDPLOT is already running!');
	return;
end

% some useful initializations
MRDS=[];
MRDS.n_subplots     = 5;
MRDS.rows           = 1;
MRDS.cols           = 1;
MRDS.freq           = 1;
MRDS.zoom           = 20;
MRDS.cursor         = 1;
MRDS.time           = toc;
MRDS.flag           = 0;
MRDS.var1 = 0;
MRDS.var2 = 0;
MRDS.string         = [];
MRDS.data           = [];
MRDS.fontsize       = 9;
MRDS.fname          = [];

% generate the command and the data window
mrdplot_command;
mrdplot_data;
return;
%----------------------------------------------------------------------------------------------
% creates the MRDPLOT command window
function mrdplot_command()

global MRDS;

s = get(0,'ScreenSize');

% small screens use smaller font sizes
if s(4) <= 600,
	MRDS.fontsize = 7;
end

fig_size(1) = s(3)-200;
fig_size(2) = 50;
fig_size(3) = 190;
fig_size(4) = s(4)-50-fig_size(2);


if ishandle(450),
	delete(450);
end;  % a pre-caution

ptr = figure(450);  % the window number is arbitrarily set to 450
set(ptr,'Units','points', ...
'Color',[0.8 0.8 0.8], ...
'NumberTitle','off', ...
'Position',fig_size, ...
'Name','MRDPLOT-Command', ...
'Resize','off',...
'Position',fig_size, ...
'Tag','Fig3');
if exist('gestalt')==3,  % indicates MacOS
	set(ptr,'WindowButtonMotionFcn','figure(MRDS.cptr);');
end
MRDS.cptr = ptr; % the pointer to the command window is stored
% in the global structure

% create the list for the variables

lptr = uicontrol('Parent',ptr, ...
'Units','points', ...
'Callback','mrdplot_functions(''varlist'');',...
'BackgroundColor',[1 1 1], ...
'Position',[10 165 fig_size(3)-20 floor(fig_size(4))-20-165], ...
'Style','listbox', ...
'Tag','List of Variables', ...
'SelectionHighlight','off',...
'Value',1);

% CGA: Got screwy fonts on cga.dsl.telerama.com Red Hat 9 so moved
% items below out of list above
%'FontSize',9,...
%'FontName','Fixed',...  % was Monaco
% try Helvetica

MRDS.lptr = lptr;  % the list pointer is also kept in the global structure
% a header for the list of variables

b = uicontrol('Parent',ptr, ...
'Units','points', ...
'BackgroundColor',[1 0.5 0], ...
'Position',[10 fig_size(4)-18 fig_size(3)-20 16], ...
'String','Variables', ...
'Style','text', ...
'Tag','Header of Variable List');

MRDS.nptr=b;

% display of time and tick
b = uicontrol('Parent',ptr, ...
'Units','points', ...
'BackgroundColor',[1 1 1], ...
'Position',[10 3 fig_size(3)-20 10], ...
'String','time=0.0  tick=0', ...
'Style','text', ...
'Tag','Time and Ticks');
MRDS.tptr=b;

% the buttons of this window

b = uicontrol('Parent',ptr, ...
'Units','points', ...
'Callback','axes(MRDS.axes(MRDS.n_subplots));title(sprintf(''%s (%s)'',MRDS.fname,datestr(now,0)));orient landscape; print -f451;',...
'BackgroundColor',[0.5 0.3 0.8], ...
'Position',[9 137 55 23], ...
'String','Print', ...
'Tag','Pushbutton1');
b = uicontrol('Parent',ptr, ...
'Units','points', ...
'Callback','axes(MRDS.axes(MRDS.n_subplots));title(sprintf(''%s (%s)'',MRDS.fname,datestr(now,0)));orient landscape; eval(sprintf(''print %s.plot -dmfile -f451'',MRDS.fname));',...
'BackgroundColor',[0.9 0.9 0.5], ...
'Position',[67 137 55 23], ...
'String','Save', ...
'Tag','Pushbutton1');
b = uicontrol('Parent',ptr, ...
'Units','points', ...
'Callback','MRDS.time=toc;MRDS.flag=7;MRDS.var1=0;MRDS.var2=0;',...
'BackgroundColor',[0.9 0.9 0.5], ...
'Position',[125 137 55 23], ...
'String','Add/Sub', ...
'Tag','Pushbutton1');

b = uicontrol('Parent',ptr, ...
'Units','points', ...
'Callback','mrdplot_functions(''open'');',...
'BackgroundColor',[0.5 0.3 0.8], ...
'Position',[9 107 55 23], ...
'String','Open ...', ...
'Tag','Pushbutton1');
b = uicontrol('Parent',ptr, ...
'Units','points', ...
'Callback','if MRDS.zoom<500,MRDS.zoom=MRDS.zoom*1.5;end;mrdplot_functions(''zoom'');',...
'BackgroundColor',[0.9 0.9 0.5], ...
'Position',[67 107 55 23], ...
'String','Zoom in', ...
'Tag','Pushbutton1');
b = uicontrol('Parent',ptr, ...
'Units','points', ...
'Callback','if MRDS.zoom>1,MRDS.zoom=MRDS.zoom/1.5;end;mrdplot_functions(''zoom'');',...
'BackgroundColor',[0.9 0.9 0.5], ...
'Position',[125 107 55 23], ...
'String','Zoom out', ...
'Tag','Pushbutton1');

b = uicontrol('Parent',ptr, ...
'Units','points', ...
'Callback','mrdplot_functions(''fast_open'');',...
'BackgroundColor',[0.5 0.3 0.8], ...
'Position',[9 77 55 23], ...
'String','Fast Open', ...
'Tag','Pushbutton1');
b = uicontrol('Parent',ptr, ...
'Units','points', ...
'Callback','MRDS.time=toc;MRDS.flag=4;MRDS.var1=0;MRDS.var2=0;',...
'BackgroundColor',[0.9 0.9 0.5], ...
'Position',[67 77 55 23], ...
'String','Phase Plot', ...
'Tag','Pushbutton1');
b = uicontrol('Parent',ptr, ...
'Units','points', ...
'Callback','MRDS.time=toc;MRDS.flag=6;',...
'BackgroundColor',[0.9 0.9 0.5], ...
'Position',[125 77 55 23], ...
'String','d/dt', ...
'Tag','Pushbutton1');

b = uicontrol('Parent',ptr, ...
'Units','points', ...
'Callback','MRDS.time=toc;MRDS.flag=2;',...
'BackgroundColor',[0.5 0.3 0.8], ...
'Position',[9 47 55 23], ...
'String','Clear', ...
'Tag','Pushbutton1');
b = uicontrol('Parent',ptr, ...
'Units','points', ...
'Callback','MRDS.time=toc;MRDS.flag=3;',...
'BackgroundColor',[0.5 0.3 0.8], ...
'Position',[67 47 55 23], ...
'String','Clear Last', ...
'Tag','Pushbutton1');
b = uicontrol('Parent',ptr, ...
'Units','points', ...
'Callback','mrdplot_functions(''clear_all'');',...
'BackgroundColor',[0.5 0.3 0.8], ...
'Position',[125 47 55 23], ...
'String','Clear All', ...
'Tag','Pushbutton1');

b = uicontrol('Parent',ptr, ...
'Units','points', ...
'Callback','mrdplot_functions(''add_subplot'');',...
'BackgroundColor',[0.5 0.3 0.8], ...
'Position',[9 17 55 23], ...
'String','Add Plot', ...
'Tag','Pushbutton1');
b = uicontrol('Parent',ptr, ...
'Units','points', ...
'Callback','mrdplot_functions(''delete_subplot'');',...
'BackgroundColor',[0.5 0.3 0.8], ...
'Position',[67 17 55 23], ...
'String','Del Plot', ...
'Tag','Pushbutton1');
b = uicontrol('Parent',ptr, ...
'Units','points', ...
'Callback','delete([MRDS.cptr,MRDS.dptr]);clear MRDS;',...
'BackgroundColor',[0.9 0.1 0.1], ...
'Position',[125 17 55 23], ...
'String','Quit', ...
'Tag','Pushbutton1');

%-------------------------------------------------------------------------------------------
function mrdplot_open(flag)
% open and reads in a file

global MRDS;

% for flag==1, we try to read the filename from the file last_data

fname =[];

if exist('last_data') & flag == 1,
	fid = fopen('last_data','r');
	fileid = fscanf(fid,'%d');
	last_data = sprintf('d%05d',fileid-1);
	fclose(fid);
	if exist(last_data),
		fname = last_data;
		pathname=' ';
	end;
end;

% read in the file name
if isempty(fname),
	[fname, pathname] = uigetfile('*','Select Data File');
	if (fname == 0),
		return;
	end;
end;

% concatenate pathname and filename and open file
fname_store = fname;
fname=strcat(pathname, fname);
fid=fopen(fname, 'r','ieee-be');
if fid == -1,
	return;
end;

specs=fscanf(fid,'%d %d %d %f',4);  % [dummy,cols,rows,freq]
MRDS.cols = specs(2);
MRDS.rows = specs(3);
MRDS.freq = specs(4);
MRDS.t    = (1:MRDS.rows)'/MRDS.freq;  % the time column
MRDS.vars = [];
MRDS.fname= fname;

% read all variable names

for i=1:MRDS.cols,
	MRDS.vars(i).name=fscanf(fid,'%s',1);
	MRDS.vars(i).unit=fscanf(fid,'%s',1);
end;
fscanf(fid,'%c',3); % there are three characters which must be skipped

% read the data
MRDS.data = fread(fid, [MRDS.cols,MRDS.rows],'float32');
MRDS.data=MRDS.data';
fclose(fid);

% build the string for showing the variable names

temp = sprintf('%s [%s]',MRDS.vars(1).name,MRDS.vars(1).unit);
string=sprintf('%20s= %% .4g',temp);
for i=2:MRDS.cols,
	temp = sprintf('%s [%s]',MRDS.vars(i).name,MRDS.vars(i).unit);
	string=[string,'|',sprintf('%20s= %% .4g',temp)];
end;

MRDS.string=string;
set(MRDS.lptr,'Value',1,'String',sprintf(MRDS.string,MRDS.data(1,:)));
MRDS.cursor = 1;
set(MRDS.tptr,'String',sprintf('time=%8.4f  tick=%5d',MRDS.t(MRDS.cursor),MRDS.cursor));

% fix all plots

mrdplot_update_subplots;

% write the file name

set(MRDS.nptr,'String',sprintf('Variables of %s',fname_store));

%------------------------------------------------------------------------------------
function mrdplot_data()
% creates the window with the data plots

global MRDS;

% get the screen size

s = get(0,'ScreenSize');
scrollbar_width = 25;

fig_size(1) = 10;
fig_size(2) = 150;
fig_size(3) = s(3)-210-fig_size(1)-scrollbar_width;
fig_size(4) = s(4)-50-fig_size(2);

if ishandle(451),
	delete(451);
end;

ptr = figure(451);   % the window number is arbitrarily set to 451
MRDS.dptr = ptr;

set(ptr,'Units','points', ...
'Color',[0.8 0.8 0.8], ...
'Name','MRDPLOT-Data', ...
'KeyPressFcn','mrdplot_functions(''key_press'');',...
'WindowButtonDownFcn','mrdplot_functions(''cursor'');',...
'NumberTitle','off', ...
'Position',fig_size, ...
'Tag','');
if exist('gestalt')==3,  % indicates MacOS
	set(ptr,'WindowButtonMotionFcn','figure(MRDS.dptr);');
end

mrdplot_create_subplots(1);

%------------------------------------------------------------------------------------------
function mrdplot_create_subplots(flag);
% create axes in data window

global MRDS;

MRDS.axes=[];
MRDS.t=(1:MRDS.rows)'/MRDS.freq;
if flag,
	MRDS.cursor=1;
end
set(MRDS.tptr,'String',sprintf('time=%8.4f  tick=%5d',MRDS.t(MRDS.cursor),MRDS.cursor));
MRDS.axes_cursors=[];
figure(MRDS.dptr);
clf;

for i=1:MRDS.n_subplots,
	
	% the main windows
	rect = [.05 0.05+(i-1)*0.95/MRDS.n_subplots 0.7 0.95/MRDS.n_subplots*0.8];
	MRDS.axes(i,1)=axes('position',rect,'XGrid','on','YGrid','on','FontSize',MRDS.fontsize);
	set(MRDS.axes(i,1),'Box','on');
	axis([0 MRDS.rows/MRDS.freq 0 1]);
	MRDS.axes_cursors(i,1) = line(MRDS.t(MRDS.cursor)*[1;1],[0;1]);
	set(MRDS.axes_cursors(i,1),'Color',[.8 .0 .0]);
	
	% the zoom window
	rect = [.8 0.05+(i-1)*0.95/MRDS.n_subplots 0.18 0.95/MRDS.n_subplots*0.8];
	MRDS.axes(i,2)=axes('position',rect,'XGrid','on','YGrid','on','FontSize',MRDS.fontsize);
	set(MRDS.axes(i,2),'Box','on');
	axis([-MRDS.rows/MRDS.freq/MRDS.zoom MRDS.rows/MRDS.freq/MRDS.zoom 0 1]);
	MRDS.axes_cursors(i,2) = line(MRDS.t(MRDS.cursor)*[1;1],[0;1]);
	set(MRDS.axes_cursors(i,2),'Color',[.8 .0 .0]);
	if flag,
		axes_data(i).values=[];
	else
		axes_data(i).values=MRDS.axes_data(i).values;		
	end;
end;

MRDS.axes_data = axes_data;

%------------------------------------------------------------------------------------------
function mrdplot_update_subplots
% create axes in data window

global MRDS;

% check whether all values are properly refered to

for i=1:MRDS.n_subplots,
	ind = find(MRDS.axes_data(i).values > MRDS.cols);
	MRDS.axes_data(i).values(ind)=[];
end;

% update the plots

MRDS.t=(1:MRDS.rows)'/MRDS.freq;

if MRDS.cursor > MRDS.rows | MRDS.cursor < 1,
	MRDS.cursor = 1;
	set(MRDS.tptr,'String',sprintf('time=%8.4f  tick=%5d',MRDS.t(MRDS.cursor),...
	MRDS.cursor));
end;

for i=1:MRDS.n_subplots,
	if isempty(MRDS.axes_data(i).values),
		clear_subplot(i);
	else
		redraw_subplot(i);
	end	
end;

%------------------------------------------------------------------------------------------
function mrdplot_zoom
% adjust the axes of zoom windows

global MRDS;

for i=1:MRDS.n_subplots,
	% the zoom window
	a=MRDS.axes(i,2);
	set(a,'XLim',[-MRDS.rows/MRDS.freq/MRDS.zoom MRDS.rows/MRDS.freq/MRDS.zoom]+MRDS.t(MRDS.cursor));
	t=get(a,'YLim');
	set(MRDS.axes_cursors(i,2),'YData',t);
end;

%------------------------------------------------------------------------------------------
function mrdplot_varlist
% processes the callback for the variable list (added to accommodate phase plots)

global MRDS;

if (MRDS.flag == 7), %---add/sub 1--
	if (toc - MRDS.time <= 6),
		MRDS.var1 = get(MRDS.lptr, 'value');
		MRDS.flag = 8;
	else,
		MRDS.time = toc;
		MRDS.flag = 1;
	end;
elseif (MRDS.flag == 8), %---add/sub 2---
	MRDS.var2 = get(MRDS.lptr, 'value');
	MRDS.flag = 0;
	MRDS.time = toc;
	ans=questdlg('Add or Subtract?','Input','+','-','-');
	if ans=='+',
		MRDS.data = [MRDS.data, MRDS.data(:,MRDS.var1)+MRDS.data(:,MRDS.var2)];
		dname = sprintf('%s+%s',MRDS.vars(MRDS.var1).name,MRDS.vars(MRDS.var2).name);
	else
		MRDS.data = [MRDS.data, MRDS.data(:,MRDS.var1)-MRDS.data(:,MRDS.var2)];
		dname = sprintf('%s-%s',MRDS.vars(MRDS.var1).name,MRDS.vars(MRDS.var2).name);
	end	
	MRDS.cols = MRDS.cols + 1;
	dunit = '-';
	dvar.name = dname;
	dvar.unit = dunit;
	MRDS.vars = [MRDS.vars, dvar];
	temp = sprintf('%s [%s]', dname, dunit);
	MRDS.string = [MRDS.string,'|',sprintf('%20s= %% .4g',temp)];
	set(MRDS.lptr,'String',sprintf(MRDS.string,MRDS.data(1,:)));
	MRDS.flag = 0;
	MRDS.time = toc;
elseif (MRDS.flag == 4), %---Phase Plot 1---
	if (toc - MRDS.time <= 6),
		MRDS.var1 = get(MRDS.lptr, 'value');
		MRDS.flag = 5;
	else,
		MRDS.time = toc;
		MRDS.flag = 1;
	end;
elseif (MRDS.flag == 5), %---Phase Plot 2---
	MRDS.var2 = get(MRDS.lptr, 'value');
	MRDS.flag = 0;
	MRDS.time = toc;
	figure;
	v1=MRDS.var1; 
	v2=MRDS.var2;
	plot(MRDS.data(:,v1),MRDS.data(:,v2));
	xl = sprintf('%s (%s)', MRDS.vars(v1).name, MRDS.vars(v1).unit);
	yl = sprintf('%s (%s)', MRDS.vars(v2).name, MRDS.vars(v2).unit);
	a=xlabel(xl,'FontSize',MRDS.fontsize);
	set(a,'Interpreter','none','FontSize',MRDS.fontsize);
	a=ylabel(yl,'FontSize',MRDS.fontsize);
	set(a,'Interpreter','none','FontSize',MRDS.fontsize);
elseif (MRDS.flag == 6), %---Differentiation---
	if (toc - MRDS.time <= 6),
		prompt = {'Filter order:','Filter cutoff frequency:'};
		def = {'2','0.2'};
		reply = inputdlg(prompt, 'Differentiation Filter Options', 1, def);
		order = sscanf(char(reply(1)),'%f');
		cutoff = sscanf(char(reply(2)), '%f');
		[b,a] = butter(order, cutoff);
		dvar = get(MRDS.lptr, 'value');
		x = diff(MRDS.data(:,dvar));
		x = [x(1); x];
		x = x * MRDS.freq;
		if order ~= 0 & cutoff ~= 1,
			filt_x = filtfilt(b, a, x);
		else 
			filt_x = x;
		end;
		MRDS.data = [MRDS.data, filt_x];
		MRDS.cols = MRDS.cols + 1;
		dname = sprintf('d%s/dt', MRDS.vars(dvar).name);
		dunit = sprintf('%s/s', MRDS.vars(dvar).unit);
		dvar.name = dname;
		dvar.unit = dunit;
		MRDS.vars = [MRDS.vars, dvar];
		temp = sprintf('%s [%s]', dname, dunit);
		MRDS.string = [MRDS.string,'|',sprintf('%20s= %% .4g',temp)];
		set(MRDS.lptr,'String',sprintf(MRDS.string,MRDS.data(1,:)));
		MRDS.flag = 0;
		MRDS.time = toc;
	else,
		MRDS.time = toc;
		MRDS.flag = 1;
	end;
else %---Normal action---
	MRDS.time=toc;
	MRDS.flag=1;
end;

%------------------------------------------------------------------------------------------
function mrdplot_cursor
% checks where the user clicked in window and adjusts cursor if necessary

global MRDS;

% was the click over a subplot?

a=overobj('axes');

if ~isempty(a) & isempty(find(MRDS.axes==a)),
	return;
end;
if ~isempty(a) & toc-MRDS.time < 5,  % if yes and only 5 secs ago the user clicked on variable,
	% the variable is added to plots
	MRDS.time = toc-6;                                                % this makes sure that a subsequent click does not cause
	% any action anymore.
	
	% which subplots?
	[i,j]=find(MRDS.axes==a);
	
	if MRDS.flag == 2 | (MRDS.flag == 3 & length(MRDS.axes_data(i).values) < 2),
		clear_subplot(i);
	elseif MRDS.flag==1 | MRDS.flag==3,  % add or delete variable
		if MRDS.flag == 1,
			% which variable is highlighted?
			v=get(MRDS.lptr,'Value');
			% add variable to the plot data
			ind=[];
			if ~isempty(MRDS.axes_data(i).values),
				ind = find(v==MRDS.axes_data(i).values);
			end;
			if isempty(ind),
				MRDS.axes_data(i).values = [MRDS.axes_data(i).values v];
				v = MRDS.axes_data(i).values;
			else,
				return;
			end;
		elseif MRDS.flag == 3,
			l = length(MRDS.axes_data(i).values);
			MRDS.axes_data(i).values(l)=[];
			v=MRDS.axes_data(i).values;
		end;
		redraw_subplot(i);
	end;
elseif ~isempty(a),
	% move the cursor
	MRDS.flag = 0;
	do_cursor;
end;

%------------------------------------------------------------------------------------------
function clear_subplot(i)
% clears a subplot

global MRDS;

MRDS.axes_data(i).values=[];
axes(MRDS.axes(i,1));
cla;
xlabel('');
MRDS.axes_cursors(i,1) = line([1;1]*MRDS.t(MRDS.cursor),[0;1]);
set(MRDS.axes_cursors(i,1),'Color',[.8 .0 .0]);
axes(MRDS.axes(i,2));
cla;
MRDS.axes_cursors(i,2) = line([1;1]*MRDS.t(MRDS.cursor),[0;1]);
set(MRDS.axes_cursors(i,2),'Color',[.8 .0 .0]);
redraw_subplot(i);

%------------------------------------------------------------------------------------------
function redraw_subplot(i)
% redraws an entire subplot

global MRDS;

% plot in full plot
v = MRDS.axes_data(i).values;

if ~isempty(v),
	axes(MRDS.axes(i,1));
	plot(MRDS.t,MRDS.data(:,v));
	set(MRDS.axes(i,1),'FontSize',MRDS.fontsize);
	grid on;
	
	% since the plot command erased the cursor, recreate it
	a=axis;
	MRDS.axes_cursors(i,1) = line([1;1]*MRDS.t(MRDS.cursor),[a(3);a(4)]);
	set(MRDS.axes_cursors(i,1),'Color',[.8 .0 .0]);
	set(MRDS.axes(i,1),'XLim',[0 MRDS.rows/MRDS.freq],'YLim',[a(3) a(4)]);
	a=ylabel(sprintf('%s   ',MRDS.vars(v).name));
	set(a,'Interpreter','none','FontSize',MRDS.fontsize);
else,
	set(MRDS.axes(i,1),'XLim',[0 MRDS.rows/MRDS.freq],'YLim',[0 1]);
	set(MRDS.axes_cursors(i,1),'XData',MRDS.t(MRDS.cursor)*[1;1],'YData',[0 1]);
end;

% plot in the zoom window
if ~isempty(v),
	axes(MRDS.axes(i,2));
	plot(MRDS.t,MRDS.data(:,v));
	set(MRDS.axes(i,2),'FontSize',MRDS.fontsize);
	grid on;
	set(MRDS.axes(i,2),'XLim',...
	[-MRDS.rows/MRDS.freq/MRDS.zoom MRDS.rows/MRDS.freq/MRDS.zoom]+MRDS.t(MRDS.cursor));
	a=axis;
	MRDS.axes_cursors(i,2) = line([1;1]*MRDS.t(MRDS.cursor),[a(3);a(4)]);
	a=axis;
	set(MRDS.axes_cursors(i,2),'Color',[.8 .0 .0],'YData',[a(3); a(4)]);
else,
	set(MRDS.axes(i,2),'XLim',[-MRDS.rows/MRDS.freq/MRDS.zoom MRDS.rows/MRDS.freq/MRDS.zoom],'YLim',[0 1]);
	set(MRDS.axes_cursors(i,2),'XData',MRDS.t(MRDS.cursor)*[1;1],'YData',[0 1]);
end;

%------------------------------------------------------------------------------------------
function do_cursor
% checks where the user clicked in window and adjusts cursor if necessary

global MRDS;

% move the cursor
a=overobj('axes');
p=get(a,'CurrentPoint');
n = round(p(1,1)*MRDS.freq);
if n > MRDS.rows,
	n = MRDS.rows;
elseif n < 1,
	n = 1;
end

if n == MRDS.cursor,
	return;
end;
MRDS.cursor = n;
update_after_cursor_move;

%------------------------------------------------------------------------------------------
function clear_all
% checks where the user clicked in window and adjusts cursor if necessary

global MRDS;

% move the cursor
for j=1:MRDS.n_subplots,
	delete(MRDS.axes(j,1));
	delete(MRDS.axes(j,2));
	MRDS.axes_data(j).values=[];
end;
mrdplot_create_subplots(1);

%------------------------------------------------------------------------------------------
function key_press
% checks where the user clicked in window and adjusts cursor if necessary

global MRDS;

% which key was pressed? f=forward b=backward

c=get(MRDS.dptr,'CurrentCharacter');

if strcmp(c,'f') & MRDS.cursor < MRDS.rows,
	MRDS.cursor = MRDS.cursor + 1;
elseif strcmp(c,'b') & MRDS.cursor > 1,
	MRDS.cursor = MRDS.cursor - 1;
else,
	return;
end;

update_after_cursor_move;

%------------------------------------------------------------------------------------------
function update_after_cursor_move
% performs necessary window updates after the cursor was moved

global MRDS;

set(MRDS.tptr,'String',sprintf('time=%8.4f  tick=%5d',MRDS.t(MRDS.cursor),MRDS.cursor));

for j=1:MRDS.n_subplots,
	set(MRDS.axes_cursors(j,1),'XData',[1;1]*MRDS.t(MRDS.cursor));
	set(MRDS.axes_cursors(j,2),'XData',[-100;-100]);
	set(MRDS.axes(j,2),'XLim',[MRDS.t(MRDS.cursor)-MRDS.rows/MRDS.freq/MRDS.zoom MRDS.t(MRDS.cursor)+MRDS.rows/MRDS.freq/MRDS.zoom]);
	% note that the repetition of the command below is necessary for proper display
	t=get(MRDS.axes(j,2),'YLim');
	set(MRDS.axes_cursors(j,2),'XData',[1;1]*MRDS.t(MRDS.cursor),'YData',t);
	t=get(MRDS.axes(j,2),'YLim');
	set(MRDS.axes_cursors(j,2),'XData',[1;1]*MRDS.t(MRDS.cursor),'YData',t);
end;

if ~isempty(MRDS.string),
	set(MRDS.lptr,'String',sprintf(MRDS.string,MRDS.data(MRDS.cursor,:)));
end;


%------------------------------------------------------------------------------------------
function add_subplot
% adds a subplot

global MRDS;

MRDS.n_subplots = MRDS.n_subplots+1;
MRDS.axes_data=[MRDS.axes_data MRDS.axes_data(1)];
MRDS.axes_data(MRDS.n_subplots).values=[];
mrdplot_create_subplots(0);
for i=1:MRDS.n_subplots,
	redraw_subplot(i);
end;
update_after_cursor_move;

%------------------------------------------------------------------------------------------
function delete_subplot
% deletes a subplot

global MRDS;

if MRDS.n_subplots > 1,
	MRDS.axes_data(MRDS.n_subplots)=[];
	MRDS.n_subplots = MRDS.n_subplots-1;
	mrdplot_create_subplots(0);
	for i=1:MRDS.n_subplots,
		redraw_subplot(i);
	end;
end;


