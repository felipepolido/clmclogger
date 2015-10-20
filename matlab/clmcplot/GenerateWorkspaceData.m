[D,vars,freq] = clmcplot_convert('file_name');
% create the named variables by looping through the data and using eval
% to assign the data to the labels
for i=1:length(vars);
        str = [vars(i).name,'=D(:,',num2str(i),');'];
        eval(str);
end

