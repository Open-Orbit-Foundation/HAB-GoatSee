import subprocess

subprocess.run("i2cdetect -y 1",shell=True)

result = subprocess.run(['./clear-i2c.sh'],capture_output=True,text=True)
print(result.stdout)