"""Timing proxy for IAD inference cost (random weights; same layer shapes as the released code).
Denoiser: TransformerEncoder d_model=512 (2*context 256), nhead=4, ff=1024, layers=4, seq=12
Refine:   TransformerEncoder d_model=128, nhead=4, ff=512, layers=2, seq=12
Per prediction: samples=20 x DDIM steps=5 (K=100, stride 20) -> 100 denoiser + 100 refine passes."""
import time, torch, torch.nn as nn, sys
torch.set_num_threads(int(sys.argv[1]) if len(sys.argv) > 1 else 2)
torch.manual_seed(0)
den = nn.TransformerEncoder(nn.TransformerEncoderLayer(512, 4, 1024), 4).eval()
ref = nn.TransformerEncoder(nn.TransformerEncoderLayer(128, 4, 512), 2).eval()
up = nn.Linear(2 + 285, 512); down = nn.Linear(512, 2); rup = nn.Linear(2 + 128, 128)
def predict(B, samples=20, steps=5, batched=False):
    with torch.inference_mode():
        if batched:
            reps = [(samples * B)]
        else:
            reps = [B] * samples
        for b in reps:
            x = torch.randn(12, b, 2)
            for _ in range(steps):
                h = up(torch.cat([x, torch.randn(12, b, 285)], -1))
                e = down(den(h))
                r = ref(rup(torch.cat([e, torch.randn(12, b, 128)], -1)))
                x = x - 0.1 * e
for B in (1, 5, 10):
    for batched in (False, True):
        predict(B, batched=batched)  # warmup
        t0 = time.perf_counter(); n = 3
        for _ in range(n): predict(B, batched=batched)
        dt = (time.perf_counter() - t0) / n
        print(f'threads={torch.get_num_threads()} pedestrians={B:2d} {"batched   " if batched else "sequential"} -> {dt*1000:7.1f} ms/prediction', flush=True)
print('params denoiser', sum(p.numel() for p in den.parameters())/1e6, 'M; refine', sum(p.numel() for p in ref.parameters())/1e6, 'M')
