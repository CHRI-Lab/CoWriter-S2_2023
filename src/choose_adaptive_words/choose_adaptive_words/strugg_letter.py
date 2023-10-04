import requests


def identify_strugg_letter(filename):

    url = "https://pen-to-print-handwriting-ocr.p.rapidapi.com/recognize/"

    files = { "srcImg": open(filename, 'rb') }
    payload = { "Session": "string" }
    headers = {
	    "X-RapidAPI-Key": "c97960a792mshf676e38f49cd3e6p12b5abjsnb145d15144af",
	    "X-RapidAPI-Host": "pen-to-print-handwriting-ocr.p.rapidapi.com"
    }

    response = requests.post(url, data=payload, files=files, headers=headers)

    return response.json()
