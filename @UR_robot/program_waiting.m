function program_waiting(obj)
    pause(0.05)
    program_running = refresh_program_status(obj);
    while program_running > 0
        program_running = refresh_program_status(obj);
        pause(0.002)
    end
end