#!/bin/sh
sudo apt update
sudo apt -y install tesseract-ocr
python -m pip install pytesseract
result=$(tesseract --version | grep "tesseract 4")
if [ -n "$result" ]
then
    echo "Tesseract successfully installed!"
else
    echo "Failed to install tesseract!"
fi