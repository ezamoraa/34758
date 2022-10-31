import collections

def info_from_qr(qr_string):
    secret_letter = {}

    qr_string = qr_string.split("\r\n")

    current_pos = qr_string[:2]

    next_pos = qr_string[2:4]

    secret_letter[int(qr_string[4][-1])] = qr_string[5][-1]

    return current_pos, next_pos, secret_letter


secret_message = {}

current_pos, next_pos, secret_letter = info_from_qr("X=2.35\r\nY=3.24\r\nX_next=5.3\r\nY_next=5.9\r\nN=3\r\nL=M")

secret_message.update(secret_letter)

if len(secret_message) != 5:
    # continue getting the next qr
    pass

else:
    print(collections.OrderedDict(sorted(secret_message.items())))


print(current_pos, next_pos, secret_letter)
print(secret_message)