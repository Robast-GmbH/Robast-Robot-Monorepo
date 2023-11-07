import 'package:flutter/material.dart';
import 'package:web_interface_flutter/constants/app_colors.dart';
import 'package:web_interface_flutter/widgets/page_frame.dart';

class AcceptOrderPage extends StatefulWidget {
  const AcceptOrderPage({super.key});

  @override
  State<AcceptOrderPage> createState() => _AcceptOrderPageState();
}

class _AcceptOrderPageState extends State<AcceptOrderPage> {
  @override
  Widget build(BuildContext context) {
    return const PageFrame(
      title: "Lieferung annehmen",
      color: AppColors.turquoise,
      child: SizedBox(),
    );
  }
}
