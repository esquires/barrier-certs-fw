import argparse
import os.path as op

from PIL import Image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('img_fnames', nargs='+')
    args = parser.parse_args()

    sz = 450
    for fname in args.img_fnames:
        img = Image.open(fname)
        w, h = img.size
        img_cropped = img.crop(
            (w / 2 - sz / 2, h / 2 - sz / 2, w / 2 + sz / 2, h / 2 + sz / 2))

        out_dir, fname_base = op.split(fname)
        fname_base, ext = op.splitext(fname_base)
        out_name = out_dir + "/" + fname_base.replace("orig", "cropped") + ext
        img_cropped.save(out_name)
    

if __name__ == '__main__':
    main()
