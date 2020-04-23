function mapped_lmk_idx  = add2map(mapped_lmk_idx, nearest_lmks_idx)
  if isempty(mapped_lmk_idx)
    disp('No mapped lmk');
    mapped_lmk_idx = nearest_lmks_idx;
  else
    assert(length(mapped_lmk_idx) >= length(nearest_lmks_idx))
    n = length(nearest_lmks_idx);
    for i = 1:n
      current_lmk_idx = nearest_lmks_idx(i);
      k = find(mapped_lmk_idx == current_lmk_idx);
      if isempty(k)
        mapped_lmk_idx = [mapped_lmk_idx; current_lmk_idx];
    end
  end
end