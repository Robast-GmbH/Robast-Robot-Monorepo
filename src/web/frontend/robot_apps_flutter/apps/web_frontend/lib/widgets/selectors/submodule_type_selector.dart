import 'package:flutter/material.dart';
import 'package:web_frontend/constants/web_colors.dart';
import 'package:web_frontend/models/controller/submodule_type_controller.dart';
import 'package:web_frontend/widgets/custom_dropdown_button.dart';
import 'package:web_frontend/widgets/rounded_container.dart';

class SubmoduleTypeSelector extends StatefulWidget {
  const SubmoduleTypeSelector({required this.controller, super.key});

  final SubmoduleTypeController controller;

  @override
  State<SubmoduleTypeSelector> createState() => _SubmoduleTypeSelectorState();
}

class _SubmoduleTypeSelectorState extends State<SubmoduleTypeSelector> {
  @override
  Widget build(BuildContext context) {
    return RoundedContainer(
        child: SizedBox(
      width: double.infinity,
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(
              'Benötigter Submodultyp',
              style: const TextStyle(fontSize: 24, color: WebColors.secondaryText),
            ),
            SizedBox(
              height: 8,
            ),
            CustomDropdownButton(
                value: widget.controller.value,
                items: ['small', 'medium', 'large'],
                onChanged: (value) {
                  setState(() {
                    widget.controller.value = value;
                  });
                })
          ],
        ),
      ),
    ));
  }
}
