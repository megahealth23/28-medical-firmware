function [greenled_arr, ecg_arr, result] = extract_ptt_data( ptt_frames, elemt_cnt)

PTT_ELEMT_SIZE = 6;

if (PTT_ELEMT_SIZE*elemt_cnt) ~= length(ptt_frames)
    result = false;
    greenled_arr = [];
    ecg_arr = [];
    return;
else
    result = true;
end

greenled_arr = [];
ecg_arr = [];
for ite = 1:elemt_cnt
    frame_sidx = (ite-1) * PTT_ELEMT_SIZE + 1;
    greenled_val = afe_convert_sensor_data( ptt_frames(frame_sidx) , ptt_frames(frame_sidx+1), ptt_frames(frame_sidx+2));
    ecg_val = afe_convert_sensor_data( ptt_frames(frame_sidx+3) , ptt_frames(frame_sidx+4),  ptt_frames(frame_sidx+5) );
   
    greenled_arr = [greenled_arr greenled_val];
    ecg_arr = [ecg_arr, ecg_val];
end
