import 'package:flutter/material.dart';
import 'package:media_kit/media_kit.dart';
import 'package:media_kit_video/media_kit_video.dart';

class VideoView extends StatefulWidget {
  const VideoView({super.key});

  @override
  State<VideoView> createState() => _VideoViewState();
}

class _VideoViewState extends State<VideoView> {
  // Create a [Player] to control playback.
  late final player = Player(configuration: const PlayerConfiguration(muted: true));
  // Create a [VideoController] to handle video output from [Player].
  late final controller = VideoController(player);
  bool startedPlaying = false;
  @override
  void initState() {
    super.initState();
    // Play a [Media] or [Playlist].
    player.open(Media('asset:///assets/robast_video.mp4'));
    player.setPlaylistMode(PlaylistMode.loop);
    // controller.notifier.addListener(() {
    //   if (controller.notifier.value. == VideoControllerState.playing) {
    //     startedPlaying = true;
    //   }
    // });
  }

  @override
  void dispose() {
    player.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Video(
      controller: controller,
    );
  }
}
