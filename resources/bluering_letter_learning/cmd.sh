GOOGLE_APPLICATION_CREDENTIALS=/home/ubuntu/catkin_ws/flowing-elf-385414-757ca2faa900.json \
  roslaunch bluering_letter_learning cowriter.launch \
  letter_model_dataset_directory:="/home/ubuntu/catkin_ws/shape_learning-pypkg/share/letter_model_datasets/bad_letters" \
  format:=wave \
  audio_topic:=/audio/audio \
  nao_handedness:=left \
  openai_timeout:=20 \
  openai_model:=gpt-3.5-turbo \
  openai_apikey:=sk-... \
  audio_outfile:="/home/ubuntu/catkin_ws/test.wav" \
  log_audio_to_file:=true \
  shape_log:="/home/ubuntu/catkin_ws/src/bluering_letter_learning/logs/23.04.18.log" \
  opoenai_max_tokens:=80 \
  audio_buflen:=20480 \
  silent_threshold:=800 \
  min_silent_chunk_to_split:=100 \
  nao_ip:=127.0.0.1