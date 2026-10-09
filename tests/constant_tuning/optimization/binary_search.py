def score_binary_search(low, high, tolerance, score_fn):
    best = low
    best_score = score_fn(low)
    curr = high

    while abs(curr-best)>tolerance:
        score = score_fn(curr)
        mid = (best+curr)/2
        if score>best_score:
            best = curr
            best_score = score
        curr = mid

    return best, best_score