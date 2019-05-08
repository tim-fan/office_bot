#!/usr/bin/env python

#node to command the robot to follow people around
#subscribes to person detections for input, and publishes to cmd_vel to control the robot

import rospy
from spencer_tracking_msgs.msg import TrackedPersons
import random 
from sound_play.libsoundplay import SoundClient


class TimesheetReminder:
    def __init__(self):
        self.lastTrackedId = None
        self.nextReminderTime = rospy.Time.from_sec(0)

        self.messages = [
            "Happy Friday",
            "Don't forget to do your timesheet",
            "Are you forgetting something?",
            "Time for timesheets",
            "It's timesheet time",
            "exterminate",
            "Do your timesheet now",
            "Don't forget",
            "Timesheets",
            "Beep boop. Do your timesheets. Boop",
            "Binary solo: One zero zero one zero one",
            "Timesheets heal all wounds",
            "It was the best of timesheets, it was the worst of timesheets",
            "I want your timesheet 5 minutes ago",
            "Critical timesheet failure",
            "Timesheet is money",
            "It's reminder time. Timesheets",
            "Stop. Timesheet time",
            "Every Friday we do our timesheets",
            "Has Kashyap done his timesheet?",
        ]
        
        self.soundhandle = SoundClient()
        
    def sayText(self, text):
        useFestival = False
        if useFestival:
            import festival
            festival.sayText(text)
        else:         
            self.soundhandle.voiceSound(text).play()
    
    def remindTimesheets(self, trackedPersonsMsg):
        """
        Given the current state of person tracking, determine the appropriate reminder
        """
    
        peopleDetected = len(trackedPersonsMsg.tracks) > 0
    
        if peopleDetected:
            #choose the track with the lowest id to follow (ie the person who was seen first)
            minTrackingId = min(track.track_id for track in trackedPersonsMsg.tracks)
            currentTrackingId = minTrackingId
    
            if self.lastTrackedId != currentTrackingId:
                self.lastTrackedId = currentTrackingId 
                self.sayText('Hello there')
                self.nextReminderTime = rospy.Time.now() + rospy.Duration(random.uniform(3,8))
            elif rospy.Time.now() > self.nextReminderTime:
                self.sayText(random.choice(self.messages))
                self.nextReminderTime = rospy.Time.now() + rospy.Duration(random.uniform(3,8))

        else:
            self.lastTrackedId = None
    
    
if __name__ == '__main__':
    
    try:
        rospy.init_node('timesheet_reminder')
        
        reminder = TimesheetReminder() 
        trackingSub = rospy.Subscriber('tracked_persons', TrackedPersons, reminder.remindTimesheets, queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
