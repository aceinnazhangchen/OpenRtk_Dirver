import os
import sys

ext_list = [".py",".h",".c",".cpp",".xml",".m",".js",".txt",".ini",".json",".doc",".docx"]

def decode_file(input_file,output_file):
    f_input = open(input_file,"rb")
    f_output = open(output_file,"wb")
    txt = f_input.read()
    f_output.write(txt)

def decode_file_one(input_file):
    encrypted_path = input_file + ".msd"
    if os.path.exists(encrypted_path):
        os.remove(encrypted_path)
    os.rename(input_file,encrypted_path)
    decode_file(encrypted_path,input_file)
    os.remove(encrypted_path)

def loop_decode_files():
    for root, dirs, files in os.walk(".", topdown=False):
        for name in files:
            ext = os.path.splitext(name)[-1]
            if ext in ext_list:
                file_path = os.path.join(root, name)
                encrypted_name = name + ".msd"
                encrypted_path = os.path.join(root, encrypted_name)
                if os.path.exists(encrypted_path):
                     os.remove(encrypted_path)
                os.rename(file_path,encrypted_path)
                decode_file(encrypted_path,file_path)
                os.remove(encrypted_path)

if __name__ == "__main__":
    if len(sys.argv) == 1:
        loop_decode_files()
    else:
        txt_file = sys.argv[1]
        decode_file_one(txt_file)