import RobotWindow from 'https://cyberbotics.com/wwi/R2025a/RobotWindow.js';

window.onload = () => {
  const robotWindow = new RobotWindow();
  robotWindow.setTitle('Contrôle du Robot Mobile');
  const send = cmd => robotWindow.send(cmd);

  // Déplacements : le robot roule tant que le bouton est maintenu
  document.querySelectorAll('[data-hold]').forEach(btn => {
    const start = e => {
      e.preventDefault();
      btn.classList.add('pressed');
      send(btn.dataset.hold);
    };
    const end = () => {
      if (!btn.classList.contains('pressed')) return;
      btn.classList.remove('pressed');
      send('stop');
    };
    btn.addEventListener('pointerdown', start);
    btn.addEventListener('pointerup', end);
    btn.addEventListener('pointerleave', end);
  });

  // Modes (formes, suivi de ligne, évitement) et stop
  const formes = document.getElementById('formes');
  document.querySelectorAll('[data-cmd]').forEach(btn => {
    btn.addEventListener('click', () => {
      send(btn.dataset.cmd);
      formes.classList.remove('open');
    });
  });

  // Le menu "Formes" s'ouvre aussi au clic (pas seulement au survol)
  document.getElementById('formesToggle').addEventListener('click', e => {
    e.stopPropagation();
    formes.classList.toggle('open');
  });
  document.addEventListener('click', () => formes.classList.remove('open'));

  // Vitesse
  const range = document.getElementById('speed-range');
  const label = document.getElementById('speed-value');
  const sendSpeed = () => {
    label.textContent = range.value;
    send('vitesse:' + range.value);
  };
  range.addEventListener('input', sendSpeed);
  sendSpeed();
};
