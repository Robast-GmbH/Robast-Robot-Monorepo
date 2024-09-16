import 'package:flutter/material.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';
import 'package:robot_frontend/widgets/video_view.dart';

class VideoPage extends StatelessWidget {
  const VideoPage({super.key});

  @override
  Widget build(BuildContext context) {
    return CustomScaffold(
      title: 'Video',
      child: Center(
          child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 32) + const EdgeInsets.only(bottom: 32, top: 16),
        child: VideoView(),
      )),
    );
  }
}
