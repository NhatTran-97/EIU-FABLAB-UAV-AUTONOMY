so = [3, 8, 14, 22]
muc_tieu = 10

# ---- Cach 1: vong for thu cong (ban dau) -------------------------
tot_nhat, diem_tot_nhat = None, None
for x in so:
    diem = abs(x - muc_tieu)
    if diem_tot_nhat is None or diem < diem_tot_nhat:
        tot_nhat = x
        diem_tot_nhat = diem
print("for   :", tot_nhat)

# ---- Cach 2: tach ham cham diem ra rieng -------------------------
def cham_diem(x):
    return abs(x - muc_tieu)

print("def   :", min(so, key=cham_diem))

# ---- Cach 3: thay def bang lambda -> mot dong --------------------
print("lambda:", min(so, key=lambda x: abs(x - muc_tieu)))
