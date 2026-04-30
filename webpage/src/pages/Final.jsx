export default function Final() {
  const pdfUrl = '/Parallel-DCD-Engine/PP_final_project.pdf'
  return (
    <div className="max-w-3xl mx-auto px-6 py-10">
      <h1>Parallel Dynamic Discrete Collision Detection Engine — Final Report</h1>
      <p style={{ color: '#666', marginTop: '.3rem', marginBottom: '1rem' }}>
        Bill Wu (shouyuaw) &amp; Shu Chen Lin (shuchen3) &nbsp;·&nbsp;
        15-618 Spring 2026 &nbsp;·&nbsp;
        <a href="https://github.com/ShuChenLin/Parallel-DCD-Engine">GitHub</a>
      </p>
      <p>
        <a href={pdfUrl} target="_blank" rel="noreferrer">
          Open PDF in new tab ↗
        </a>
        &nbsp;·&nbsp;
        <a href={pdfUrl} download>
          Download
        </a>
      </p>
      <iframe
        src={pdfUrl}
        title="Final Report"
        style={{ width: '100%', height: '90vh', border: '1px solid #ddd', marginTop: '1rem' }}
      />
    </div>
  )
}
