function info = student_info()
%STUDENT_INFO Student identity used by the automated grader.

    info.student_id = '202220882';

    % ASCII-safe representation of the Korean name "Park Gyeong-min".
    info.name = char([48149 44221 48124]);

    info.team_members = {};
    info.course = 'Automatic Control - 2026 Spring';
    info.ai_usage = ['OpenAI Codex was used for repository inspection, ', ...
                     'controller scaffolding, preliminary tuning support, ', ...
                     'and report drafting. Final MATLAB execution and ', ...
                     'numeric validation were performed by the student.'];

    if contains(info.student_id, 'TODO_FILL')
        warning('[student_info] Student ID is not filled in.');
    end
    if contains(info.name, 'TODO_FILL')
        warning('[student_info] Student name is not filled in.');
    end
end
