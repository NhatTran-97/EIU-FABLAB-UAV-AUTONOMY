

so = [3, 8, 14, 22]
muc_tieu = 10

tot_nhat, diem_tot_nhat = None, None

for x in so:                       # x = BIẾN CHẠY, nhận lần lượt 3, 8, 14, 22
    diem = abs(x - muc_tieu)       # chấm điểm cho x
    if diem_tot_nhat is None or diem < diem_tot_nhat:
        tot_nhat = x               # nhớ lại người thắng  3
        diem_tot_nhat = diem                           # 2
print(tot_nhat)                    # 8


def cham_diem(x):
    return abs(x - muc_tieu)

min(so, key=cham_diem)        # 8


min(so, key=lambda x: abs(x - muc_tieu))     # 8
