﻿using System.Xml.Linq;
using System;
using System.ComponentModel;
using OptiSort.userControls;

namespace OptiSort.Properties {
    
    
    // This class allows you to handle specific events on the settings class:
    //  The SettingChanging event is raised before a setting's value is changed.
    //  The PropertyChanged event is raised after a setting's value is changed.
    //  The SettingsLoaded event is raised after the setting values are loaded.
    //  The SettingsSaving event is raised before the setting values are saved.
    internal sealed partial class Settings {
        
        public Settings() {
            //this.SettingChanging += this.SettingChangingEventHandler;
            //this.PropertyChanged += this.PropertyChangedEventHandler;
            //this.SettingsSaving += this.SettingsSavingEventHandler;
        }

        private void PropertyChangedEventHandler(object sender, PropertyChangedEventArgs e)
        {
            
        }

        private void SettingChangingEventHandler(object sender, System.Configuration.SettingChangingEventArgs e) {

        }
        
        private void SettingsSavingEventHandler(object sender, System.ComponentModel.CancelEventArgs e) {
            
        }
    }
}
