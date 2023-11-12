function force_mode(obj,selection_vector,wrench,limits)

command = sprintf('force_mode(p[0.0,0.0,0.0,0.0,0.0,0.0],[%f,%f,%f,%f,%f,%f],[%f,%f,%f,%f,%f,%f],2,[%f,%f,%f,%f,%f,%f])\n',...
    selection_vector,wrench,limits);

fprintf(obj.s2,command);

end