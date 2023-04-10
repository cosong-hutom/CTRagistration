# CTRagistration
python version : 3.9.6 <br>
library : requirements.txt 참조 <br>
<br>
<br>
<br>
<h2> 파일 설명 </h2>
- registrationStep.py <br>
EAP nifti 파일들에 대해 registration을 수행한다. <br>
형식은 다음과 같다. <br>
<br><pre>
    -eap : EAP nifti 파일들이 저장되어 있는 FullPath를 지정 
    -pp  : PP nifti 파일들이 저장되어 있는 FullPath를 지정
    -o   : Registration 및 Resampling이 된 EAP nifti 파일들을 복사 할 FullPath를 지정
</pre><br>
ex) <br>
<pre>python registrationStep.py -eap "~/01009ug_9/eap" -pp "~/01009ug_9/pp" -o "~/01009ug_9/eapReg" </pre><br>
<br>
<br>
<h2> 주의 사항 </h2>
- 지정 된 eap 폴더에는 반드시 CT.nii.gz 파일이 있어야 된다. <br>
- 지정 된 pp 폴더에는 반드시 VA.nii.gz 파일이 있어야 된다. <br>


