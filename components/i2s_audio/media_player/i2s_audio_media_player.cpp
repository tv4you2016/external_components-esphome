#include "i2s_audio_media_player.h"

#ifdef USE_ESP32_FRAMEWORK_ARDUINO

#include "esphome/core/log.h"

namespace esphome {
namespace i2s_audio {

static const char *const TAG = "audio";
static const uint8_t GAIN[] = {0xCF};

void I2SAudioMediaPlayer::control(const media_player::MediaPlayerCall &call) {
  media_player::MediaPlayerState play_state = media_player::MEDIA_PLAYER_STATE_PLAYING;
  if (call.get_announcement().has_value()) {
    play_state = call.get_announcement().value() ? media_player::MEDIA_PLAYER_STATE_ANNOUNCING
                                                 : media_player::MEDIA_PLAYER_STATE_PLAYING;
  }
  if (call.get_media_url().has_value()) {
    this->current_url_ = call.get_media_url();
    if (this->i2s_state_ != I2S_STATE_STOPPED && this->audio_ != nullptr) {
      if (this->audio_->isRunning()) {
        this->audio_->stopSong();
      }
      this->audio_->connecttohost(this->current_url_.value().c_str());
      this->state = play_state;
    } else {
      this->start();
    }
  }

  if (play_state == media_player::MEDIA_PLAYER_STATE_ANNOUNCING) {
    this->is_announcement_ = true;
  }

  if (call.get_volume().has_value()) {
    this->volume = call.get_volume().value();
    this->set_volume_(volume);
    this->unmute_();
  }
  if (call.get_command().has_value()) {
    switch (call.get_command().value()) {
      case media_player::MEDIA_PLAYER_COMMAND_MUTE:
        this->mute_();
        break;
      case media_player::MEDIA_PLAYER_COMMAND_UNMUTE:
        this->unmute_();
        break;
      case media_player::MEDIA_PLAYER_COMMAND_VOLUME_UP: {
        float new_volume = this->volume + 0.1f;
        if (new_volume > 1.0f)
          new_volume = 1.0f;
        this->set_volume_(new_volume);
        this->unmute_();
        break;
      }
      case media_player::MEDIA_PLAYER_COMMAND_VOLUME_DOWN: {
        float new_volume = this->volume - 0.1f;
        if (new_volume < 0.0f)
          new_volume = 0.0f;
        this->set_volume_(new_volume);
        this->unmute_();
        break;
      }
      default:
        break;
    }
    if (this->i2s_state_ != I2S_STATE_RUNNING) {
      return;
    }
    switch (call.get_command().value()) {
      case media_player::MEDIA_PLAYER_COMMAND_PLAY:
        if (!this->audio_->isRunning())
          this->audio_->pauseResume();
        this->state = play_state;
        break;
      case media_player::MEDIA_PLAYER_COMMAND_PAUSE:
        if (this->audio_->isRunning())
          this->audio_->pauseResume();
        this->state = media_player::MEDIA_PLAYER_STATE_PAUSED;
        break;
      case media_player::MEDIA_PLAYER_COMMAND_STOP:
        this->stop();
        break;
      case media_player::MEDIA_PLAYER_COMMAND_TOGGLE:
        this->audio_->pauseResume();
        if (this->audio_->isRunning()) {
          this->state = media_player::MEDIA_PLAYER_STATE_PLAYING;
        } else {
          this->state = media_player::MEDIA_PLAYER_STATE_PAUSED;
        }
        break;
      default:
        break;
    }
  }
  this->publish_state();
}

void I2SAudioMediaPlayer::mute_() {
  if (this->mute_pin_ != nullptr) {
    this->mute_pin_->digital_write(true);
  } else {
    this->set_volume_(0.0f, false);
  }
  this->muted_ = true;
}
void I2SAudioMediaPlayer::unmute_() {
  if (this->mute_pin_ != nullptr) {
    this->mute_pin_->digital_write(false);
  } else {
    this->set_volume_(this->volume, false);
  }
  this->muted_ = false;
}
void I2SAudioMediaPlayer::set_volume_(float volume, bool publish) {
  if (this->audio_ != nullptr)
    this->audio_->setVolume(remap<uint8_t, float>(volume, 0.0f, 1.0f, 0, 21));
  if (publish)
    this->volume = volume;
}

void I2SAudioMediaPlayer::setup() {
  ESP_LOGCONFIG(TAG, "Running setup");
  this->state = media_player::MEDIA_PLAYER_STATE_IDLE;
}

void I2SAudioMediaPlayer::loop() {
  switch (this->i2s_state_) {
    case I2S_STATE_STARTING:
      this->start_();
      break;
    case I2S_STATE_RUNNING:
      this->play_();
      break;
    case I2S_STATE_STOPPING:
      this->stop_();
      break;
    case I2S_STATE_STOPPED:
      break;
  }
}

void I2SAudioMediaPlayer::play_() {
  this->audio_->loop();
  if ((this->state == media_player::MEDIA_PLAYER_STATE_PLAYING ||
       this->state == media_player::MEDIA_PLAYER_STATE_ANNOUNCING) &&
      !this->audio_->isRunning()) {
    this->stop();
  }
}

void I2SAudioMediaPlayer::start() { this->i2s_state_ = I2S_STATE_STARTING; }
void I2SAudioMediaPlayer::start_() {
  
  if (!this->write_bytes(0x04, GAIN, sizeof(GAIN))) {
    ESP_LOGE(TAG, "GAIN 0x04 failed!");
    this->mark_failed();
    return;
  } else
    ESP_LOGCONFIG(TAG, "GAIN OK...");


  if (!this->write_bytes(0x05, GAIN, sizeof(GAIN))) {
    ESP_LOGE(TAG, "GAIN 0x05 failed!");
    this->mark_failed();
    return;
  } else
    ESP_LOGCONFIG(TAG, "GAIN OK...");


  if (this->is_failed()) {
    ESP_LOGCONFIG(TAG, "Audio failed to initialize!");
    return;
  }

  if (!this->parent_->try_lock()) {
    return;  // Waiting for another i2s to return lock
  }

#if SOC_I2S_SUPPORTS_DAC
  if (this->internal_dac_mode_ != I2S_DAC_CHANNEL_DISABLE) {
    this->audio_ = make_unique<Audio>(this->parent_->get_port());
  } else {
#endif
    this->audio_ = make_unique<Audio>();

    i2s_pin_config_t pin_config = this->parent_->get_pin_config();
    pin_config.data_out_num = this->dout_pin_;

    ESP_LOGCONFIG(TAG,"bck_io_num: %d  ws_io_num: %d data_out_num: %d,data_in_num: %d \n",pin_config.bck_io_num, pin_config.ws_io_num,pin_config.data_out_num, pin_config.data_in_num);
    //i2s_set_pin(this->parent_->get_port(), &pin_config);
    this->audio_->setPinout(pin_config.bck_io_num, pin_config.ws_io_num,pin_config.data_out_num,pin_config.mck_io_num);
  


    
    if (this->mute_pin_ != nullptr) {
      this->mute_pin_->setup();
      this->mute_pin_->digital_write(false);
    }
#if SOC_I2S_SUPPORTS_DAC
  }
#endif

  this->i2s_state_ = I2S_STATE_RUNNING;
  this->high_freq_.start();
  this->audio_->setVolume(remap<uint8_t, float>(this->volume, 0.0f, 1.0f, 0, 21));
  if (this->current_url_.has_value()) {
    this->audio_->connecttohost(this->current_url_.value().c_str());
    this->state = media_player::MEDIA_PLAYER_STATE_PLAYING;
    if (this->is_announcement_) {
      this->state = media_player::MEDIA_PLAYER_STATE_ANNOUNCING;
    }
    this->publish_state();
  }
}
void I2SAudioMediaPlayer::stop() {
  if (this->i2s_state_ == I2S_STATE_STOPPED) {
    return;
  }
  if (this->i2s_state_ == I2S_STATE_STARTING) {
    this->i2s_state_ = I2S_STATE_STOPPED;
    return;
  }
  this->i2s_state_ = I2S_STATE_STOPPING;
}
void I2SAudioMediaPlayer::stop_() {
  if (this->audio_->isRunning()) {
    this->audio_->stopSong();
    return;
  }

  this->audio_ = nullptr;
  this->current_url_ = {};
  this->parent_->unlock();
  this->i2s_state_ = I2S_STATE_STOPPED;

  this->high_freq_.stop();
  this->state = media_player::MEDIA_PLAYER_STATE_IDLE;
  this->publish_state();
  this->is_announcement_ = false;
}

media_player::MediaPlayerTraits I2SAudioMediaPlayer::get_traits() {
  auto traits = media_player::MediaPlayerTraits();
  traits.set_supports_pause(true);
  return traits;
};

void I2SAudioMediaPlayer::dump_config() {
  ESP_LOGCONFIG(TAG, "Audio:");
  if (this->is_failed()) {
    ESP_LOGCONFIG(TAG, "Audio failed to initialize!");
    return;
  }
#if SOC_I2S_SUPPORTS_DAC
  if (this->internal_dac_mode_ != I2S_DAC_CHANNEL_DISABLE) {
    switch (this->internal_dac_mode_) {
      case I2S_DAC_CHANNEL_LEFT_EN:
        ESP_LOGCONFIG(TAG, "  Internal DAC mode: Left");
        break;
      case I2S_DAC_CHANNEL_RIGHT_EN:
        ESP_LOGCONFIG(TAG, "  Internal DAC mode: Right");
        break;
      case I2S_DAC_CHANNEL_BOTH_EN:
        ESP_LOGCONFIG(TAG, "  Internal DAC mode: Left & Right");
        break;
      default:
        break;
    }
  } else {
#endif
    ESP_LOGCONFIG(TAG,
                  "  External DAC channels: %d\n"
                  "  I2S DOUT Pin: %d",
                  this->external_dac_channels_, this->dout_pin_);
    LOG_PIN("  Mute Pin: ", this->mute_pin_);
#if SOC_I2S_SUPPORTS_DAC
  }
#endif
}

}  // namespace i2s_audio
}  // namespace esphome

#endif  // USE_ESP32_FRAMEWORK_ARDUINO
