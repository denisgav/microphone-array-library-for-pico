To play use following command:
sudo chmod 666 /dev/ttyACM0
cat /dev/ttyACM0 | xxd -r -p | aplay -r16000 -c1 -fS32_BE