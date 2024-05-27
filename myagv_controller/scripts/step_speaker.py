#!/usr/bin/env python3

import rospy
import


if self.step != self.previous_step:
                if self.step == 0:
                    self.play_mp3_and_log(step0_map3_path)
                elif self.step == 10:
                    self.play_mp3_and_log(step10_mp3_path)
                elif self.step == 30:
                    self.play_mp3_and_log(step30_mp3_path)
                elif self.step == 40:
                    self.play_mp3_and_log(step40_mp3_path)
                elif self.step == 50:
                    self.play_mp3_and_log(step50_mp3_path)
                elif self.step == 60:
                    self.play_mp3_and_log(step60_mp3_path)
            self.previous_step = self.step  # 현재 스텝을 이전 스텝으로 저장