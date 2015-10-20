function index=findMRDPLOTindex(names, varName)

index=0;
i=1;
while index== 0
   if(strcmp(deblank(char(names(i,:))), varName))
      index=i;
   end
   i=i+1;
end

return;