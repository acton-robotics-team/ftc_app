package com.github.actonroboticsteam.ftcapp;

import android.content.Context;
import android.media.AudioManager;
import android.media.SoundPool;

import java.util.HashMap;
import java.util.Map;

/**
 * Created by kunal on 12/11/2017.
 */

public class PlaySongs {

    // Creating the sound files (uncomment when needed)
    //public static final int S1 = R.raw.s1;
    //public static final int S2 = R.raw.s2;
    //public static final int S3 = R.raw.s3;

    // Storage of all the sounds
    private static SoundPool soundPool;
    private static Map<Integer, Integer> soundPoolMap;

    // Init all of the sounds lelllll
    public static void initSounds(Context context) {
        soundPool = new SoundPool(2, AudioManager.STREAM_MUSIC, 100);
        soundPoolMap = new HashMap<>(3);
        //soundPoolMap.put(S1, soundPool.load(context, R.raw.s1, 1));
        //soundPoolMap.put(S2, soundPool.load(context, R.raw.s2, 2));
        //soundPoolMap.put(S3, soundPool.load(context, R.raw.s3, 3));
        // uncomment when putting in sounds

    }

    // The actual function to play a sound
    public static void playSound(Context context, int soundID) {
        if (soundPool == null || soundPoolMap == null) {
            initSounds(context);
        }
        float volume = 8; // Can be in the range of 0.0 -> 10.0 (low -> high)

        // play sound with same right and left volume, with a priority of 1
        soundPool.play(soundPoolMap.get(soundID), volume, volume, 1, 0, 1f);
    }
}

